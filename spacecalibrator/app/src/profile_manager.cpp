#include <spacecal/app/profile_manager.h>
#include <spacecal/core/calibration_policy.h>

#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>

namespace spacecal {

namespace {

struct JsonValue {
    enum class Type { Null, Boolean, Number, String, Array, Object };
    using Array = std::vector<JsonValue>;
    using Object = std::map<std::string, JsonValue>;

    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string string;
    Array array;
    Object object;

    static JsonValue null() { return {}; }
    static JsonValue booleanValue(bool value) {
        JsonValue v;
        v.type = Type::Boolean;
        v.boolean = value;
        return v;
    }
    static JsonValue numberValue(double value) {
        JsonValue v;
        v.type = Type::Number;
        v.number = value;
        return v;
    }
    static JsonValue stringValue(std::string value) {
        JsonValue v;
        v.type = Type::String;
        v.string = std::move(value);
        return v;
    }
    static JsonValue arrayValue(Array value) {
        JsonValue v;
        v.type = Type::Array;
        v.array = std::move(value);
        return v;
    }
    static JsonValue objectValue(Object value) {
        JsonValue v;
        v.type = Type::Object;
        v.object = std::move(value);
        return v;
    }

    bool isArray() const { return type == Type::Array; }
    bool isObject() const { return type == Type::Object; }
    bool isString() const { return type == Type::String; }
    bool isNumber() const { return type == Type::Number; }
};

class JsonParser {
public:
    explicit JsonParser(const std::string& input) : input_(input) {}

    JsonValue parse() {
        auto value = parseValue();
        skipWhitespace();
        if (pos_ != input_.size()) {
            fail("Unexpected trailing data");
        }
        return value;
    }

private:
    JsonValue parseValue() {
        skipWhitespace();
        if (pos_ >= input_.size()) {
            fail("Unexpected end of input");
        }

        const char c = input_[pos_];
        if (c == 'n') return parseLiteral("null", JsonValue::null());
        if (c == 't') return parseLiteral("true", JsonValue::booleanValue(true));
        if (c == 'f') return parseLiteral("false", JsonValue::booleanValue(false));
        if (c == '"') return JsonValue::stringValue(parseString());
        if (c == '[') return parseArray();
        if (c == '{') return parseObject();
        if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
            return parseNumber();
        }

        fail("Unexpected JSON value");
    }

    JsonValue parseLiteral(const char* literal, JsonValue value) {
        const std::string expected(literal);
        if (input_.compare(pos_, expected.size(), expected) != 0) {
            fail("Invalid literal");
        }
        pos_ += expected.size();
        return value;
    }

    JsonValue parseNumber() {
        const char* start = input_.c_str() + pos_;
        char* end = nullptr;
        errno = 0;
        const double value = std::strtod(start, &end);
        if (end == start) {
            fail("Invalid number");
        }
        if (errno == ERANGE) {
            fail("Number out of range");
        }
        if (!std::isfinite(value)) {
            fail("Non-finite number");
        }
        pos_ += static_cast<size_t>(end - start);
        return JsonValue::numberValue(value);
    }

    JsonValue parseArray() {
        consume('[');
        JsonValue::Array values;
        skipWhitespace();
        if (tryConsume(']')) {
            return JsonValue::arrayValue(std::move(values));
        }

        while (true) {
            values.push_back(parseValue());
            skipWhitespace();
            if (tryConsume(']')) {
                return JsonValue::arrayValue(std::move(values));
            }
            consume(',');
        }
    }

    JsonValue parseObject() {
        consume('{');
        JsonValue::Object values;
        skipWhitespace();
        if (tryConsume('}')) {
            return JsonValue::objectValue(std::move(values));
        }

        while (true) {
            skipWhitespace();
            if (pos_ >= input_.size() || input_[pos_] != '"') {
                fail("Expected object key");
            }
            auto key = parseString();
            skipWhitespace();
            consume(':');
            values[std::move(key)] = parseValue();
            skipWhitespace();
            if (tryConsume('}')) {
                return JsonValue::objectValue(std::move(values));
            }
            consume(',');
        }
    }

    std::string parseString() {
        consume('"');
        std::string output;
        while (pos_ < input_.size()) {
            const unsigned char c = static_cast<unsigned char>(input_[pos_++]);
            if (c == '"') {
                return output;
            }
            if (c < 0x20) {
                fail("Unescaped control character in string");
            }
            if (c != '\\') {
                output.push_back(static_cast<char>(c));
                continue;
            }

            if (pos_ >= input_.size()) {
                fail("Unterminated escape sequence");
            }
            const char escaped = input_[pos_++];
            switch (escaped) {
                case '"': output.push_back('"'); break;
                case '\\': output.push_back('\\'); break;
                case '/': output.push_back('/'); break;
                case 'b': output.push_back('\b'); break;
                case 'f': output.push_back('\f'); break;
                case 'n': output.push_back('\n'); break;
                case 'r': output.push_back('\r'); break;
                case 't': output.push_back('\t'); break;
                case 'u': parseUnicodeEscape(output); break;
                default: fail("Invalid escape sequence");
            }
        }

        fail("Unterminated string");
    }

    void parseUnicodeEscape(std::string& output) {
        uint32_t codepoint = parseHex4();
        if (0xD800 <= codepoint && codepoint <= 0xDBFF) {
            if (pos_ + 1 >= input_.size() || input_[pos_] != '\\' || input_[pos_ + 1] != 'u') {
                fail("Missing low surrogate");
            }
            pos_ += 2;
            const uint32_t low = parseHex4();
            if (low < 0xDC00 || low > 0xDFFF) {
                fail("Invalid low surrogate");
            }
            codepoint = 0x10000 + ((codepoint - 0xD800) << 10) + (low - 0xDC00);
        }
        appendUtf8(output, codepoint);
    }

    uint32_t parseHex4() {
        if (pos_ + 4 > input_.size()) {
            fail("Incomplete unicode escape");
        }

        uint32_t value = 0;
        for (int i = 0; i < 4; i++) {
            const char c = input_[pos_++];
            value <<= 4;
            if ('0' <= c && c <= '9') value += static_cast<uint32_t>(c - '0');
            else if ('a' <= c && c <= 'f') value += static_cast<uint32_t>(c - 'a' + 10);
            else if ('A' <= c && c <= 'F') value += static_cast<uint32_t>(c - 'A' + 10);
            else fail("Invalid unicode escape");
        }
        return value;
    }

    static void appendUtf8(std::string& output, uint32_t codepoint) {
        if (codepoint <= 0x7F) {
            output.push_back(static_cast<char>(codepoint));
        } else if (codepoint <= 0x7FF) {
            output.push_back(static_cast<char>(0xC0 | (codepoint >> 6)));
            output.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
        } else if (codepoint <= 0xFFFF) {
            output.push_back(static_cast<char>(0xE0 | (codepoint >> 12)));
            output.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
            output.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
        } else if (codepoint <= 0x10FFFF) {
            output.push_back(static_cast<char>(0xF0 | (codepoint >> 18)));
            output.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3F)));
            output.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
            output.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
        } else {
            throw std::runtime_error("Invalid unicode codepoint");
        }
    }

    void skipWhitespace() {
        while (pos_ < input_.size() &&
               std::isspace(static_cast<unsigned char>(input_[pos_]))) {
            pos_++;
        }
    }

    bool tryConsume(char expected) {
        skipWhitespace();
        if (pos_ < input_.size() && input_[pos_] == expected) {
            pos_++;
            return true;
        }
        return false;
    }

    void consume(char expected) {
        skipWhitespace();
        if (pos_ >= input_.size() || input_[pos_] != expected) {
            std::string msg = "Expected '";
            msg.push_back(expected);
            msg.push_back('\'');
            fail(msg);
        }
        pos_++;
    }

    [[noreturn]] void fail(const std::string& message) const {
        throw std::runtime_error(message + " at byte " + std::to_string(pos_));
    }

    const std::string& input_;
    size_t pos_ = 0;
};

static void writeJsonString(std::ostringstream& out, const std::string& value) {
    out << '"';
    for (const unsigned char c : value) {
        switch (c) {
            case '"': out << "\\\""; break;
            case '\\': out << "\\\\"; break;
            case '\b': out << "\\b"; break;
            case '\f': out << "\\f"; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if (c < 0x20) {
                    out << "\\u"
                        << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c)
                        << std::dec << std::setfill(' ');
                } else {
                    out << static_cast<char>(c);
                }
                break;
        }
    }
    out << '"';
}

static void writeJsonNumber(std::ostringstream& out, double value) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument("Cannot serialize non-finite JSON number");
    }
    out << std::setprecision(17) << value;
}

static void writeFieldName(std::ostringstream& out, bool& first, const char* key) {
    if (!first) {
        out << ',';
    }
    first = false;
    writeJsonString(out, key);
    out << ':';
}

static void writeNumberField(std::ostringstream& out, bool& first, const char* key, double value) {
    writeFieldName(out, first, key);
    writeJsonNumber(out, value);
}

static void writeBoolField(std::ostringstream& out, bool& first, const char* key, bool value) {
    writeFieldName(out, first, key);
    out << (value ? "true" : "false");
}

static void writeStringField(
    std::ostringstream& out,
    bool& first,
    const char* key,
    const std::string& value
) {
    writeFieldName(out, first, key);
    writeJsonString(out, value);
}

static void writeFloatArray(std::ostringstream& out, const float* buf, size_t count) {
    out << '[';
    for (size_t i = 0; i < count; i++) {
        if (i > 0) {
            out << ',';
        }
        writeJsonNumber(out, static_cast<double>(buf[i]));
    }
    out << ']';
}

static const JsonValue* findValue(const JsonValue::Object& obj, const char* key) {
    auto it = obj.find(key);
    return it == obj.end() ? nullptr : &it->second;
}

static void getString(const JsonValue::Object& obj, const char* key, std::string& out) {
    const auto* value = findValue(obj, key);
    if (value && value->isString()) {
        out = value->string;
    }
}

static void getNumber(const JsonValue::Object& obj, const char* key, double& out) {
    const auto* value = findValue(obj, key);
    if (value && value->isNumber()) {
        out = value->number;
    }
}

static bool evaluateAsBoolean(const JsonValue& value) {
    switch (value.type) {
        case JsonValue::Type::Boolean: return value.boolean;
        case JsonValue::Type::Number: return value.number != 0.0;
        case JsonValue::Type::String: return !value.string.empty();
        case JsonValue::Type::Array: return !value.array.empty();
        case JsonValue::Type::Object: return !value.object.empty();
        case JsonValue::Type::Null: return false;
    }
    return false;
}

static bool getBoolean(const JsonValue::Object& obj, const char* key) {
    const auto* value = findValue(obj, key);
    return value ? evaluateAsBoolean(*value) : false;
}

static void tryGetBoolean(const JsonValue::Object& obj, const char* key, bool& out) {
    const auto* value = findValue(obj, key);
    if (value) {
        out = evaluateAsBoolean(*value);
    }
}

} // namespace

// ============================================================================
// ProfileManager
// ============================================================================

ProfileManager::ProfileManager(
    std::unique_ptr<platform::IConfigStore> store,
    std::unique_ptr<IProfileSerializer> serializer)
    : store_(std::move(store))
    , serializer_(std::move(serializer))
{}

Expected<CalibrationProfile> ProfileManager::loadActive()
{
    auto data = store_->load(kActiveProfileKey);
    if (!data)
        return data.error();

    return serializer_->deserialize(data.value());
}

Expected<void> ProfileManager::saveActive(const CalibrationProfile& profile)
{
    std::string data = serializer_->serialize(profile);
    return store_->save(kActiveProfileKey, data);
}

// ============================================================================
// JsonProfileSerializer helpers
// ============================================================================

static void writeAlignmentParams(
    std::ostringstream& out,
    const AlignmentSpeedParams& p,
    double continuousCalibrationThreshold
)
{
    bool first = true;
    out << '{';
    writeNumberField(out, first, "align_speed_tiny", p.align_speed_tiny);
    writeNumberField(out, first, "align_speed_small", p.align_speed_small);
    writeNumberField(out, first, "align_speed_large", p.align_speed_large);
    writeNumberField(out, first, "thr_trans_tiny", p.thr_trans_tiny);
    writeNumberField(out, first, "thr_trans_small", p.thr_trans_small);
    writeNumberField(out, first, "thr_trans_large", p.thr_trans_large);
    writeNumberField(out, first, "thr_rot_tiny", p.thr_rot_tiny);
    writeNumberField(out, first, "thr_rot_small", p.thr_rot_small);
    writeNumberField(out, first, "thr_rot_large", p.thr_rot_large);
    writeNumberField(out, first, "continuousCalibrationThreshold", continuousCalibrationThreshold);
    out << '}';
}

static void writeDeviceId(std::ostringstream& out, const CalibrationProfile::DeviceId& d)
{
    bool first = true;
    out << '{';
    writeStringField(out, first, "tracking_system", d.trackingSystem);
    writeStringField(out, first, "model", d.model);
    writeStringField(out, first, "serial", d.serial);
    out << '}';
}

static void loadFloatArray(const JsonValue& v, float* buf, int count)
{
    if (!v.isArray()) return;
    const auto& arr = v.array;
    for (int i = 0; i < count && i < static_cast<int>(arr.size()); i++)
        if (arr[i].isNumber())
            buf[i] = static_cast<float>(arr[i].number);
}

// ============================================================================
// JsonProfileSerializer::serialize
// ============================================================================

std::string JsonProfileSerializer::serialize(const CalibrationProfile& profile)
{
    std::ostringstream out;
    bool first = true;

    // Wrap in array for legacy format compatibility.
    out << "[{";
    writeNumberField(out, first, "version", static_cast<double>(profile.version));
    writeStringField(out, first, "name", profile.name);

    // Continuous calibration threshold is stored alongside alignment params in
    // the legacy format. Mirror that behavior.
    writeFieldName(out, first, "alignment_params");
    writeAlignmentParams(out, profile.alignmentParams, profile.continuousCalibrationThreshold);

    writeStringField(out, first, "reference_tracking_system", profile.referenceTrackingSystem);
    writeStringField(out, first, "target_tracking_system", profile.targetTrackingSystem);

    // eulerRotationDegrees: (0)=roll, (1)=yaw, (2)=pitch
    writeNumberField(out, first, "roll", profile.eulerRotationDegrees(0));
    writeNumberField(out, first, "yaw", profile.eulerRotationDegrees(1));
    writeNumberField(out, first, "pitch", profile.eulerRotationDegrees(2));

    // translationCm: (0)=x, (1)=y, (2)=z
    writeNumberField(out, first, "x", profile.translationCm(0));
    writeNumberField(out, first, "y", profile.translationCm(1));
    writeNumberField(out, first, "z", profile.translationCm(2));

    writeNumberField(out, first, "scale", profile.scale);

    writeFieldName(out, first, "reference_device");
    writeDeviceId(out, profile.referenceDevice);
    writeFieldName(out, first, "target_device");
    writeDeviceId(out, profile.targetDevice);

    writeBoolField(out, first, "autostart_continuous_calibration", profile.autostartContinuous);
    writeBoolField(out, first, "enable_static_recalibration", profile.enableStaticRecalibration);
    writeBoolField(out, first, "quash_target_in_continuous", profile.quashTargetInContinuous);
    writeNumberField(out, first, "calibration_speed", static_cast<double>(static_cast<int>(profile.speed)));

    if (profile.chaperone.has_value()) {
        const auto& ch = profile.chaperone.value();
        bool chFirst = true;
        writeFieldName(out, first, "chaperone");
        out << '{';
        writeBoolField(out, chFirst, "auto_apply", ch.autoApply);
        writeFieldName(out, chFirst, "play_space_size");
        writeFloatArray(out, ch.playSpaceSize, 2);
        writeFieldName(out, chFirst, "standing_center");
        writeFloatArray(out, ch.standingCenter, 12);
        writeFieldName(out, chFirst, "geometry");
        writeFloatArray(out, ch.geometryData.data(), ch.geometryData.size());
        out << '}';
    }

    out << "}]";
    return out.str();
}

// ============================================================================
// JsonProfileSerializer::deserialize
// ============================================================================

static AlignmentSpeedParams loadAlignmentParams(const JsonValue& v)
{
    AlignmentSpeedParams p;
    if (!v.isObject()) return p;
    const auto& obj = v.object;

    getNumber(obj, "align_speed_tiny",  p.align_speed_tiny);
    getNumber(obj, "align_speed_small", p.align_speed_small);
    getNumber(obj, "align_speed_large", p.align_speed_large);
    getNumber(obj, "thr_trans_tiny",    p.thr_trans_tiny);
    getNumber(obj, "thr_trans_small",   p.thr_trans_small);
    getNumber(obj, "thr_trans_large",   p.thr_trans_large);
    getNumber(obj, "thr_rot_tiny",      p.thr_rot_tiny);
    getNumber(obj, "thr_rot_small",     p.thr_rot_small);
    getNumber(obj, "thr_rot_large",     p.thr_rot_large);
    return p;
}

static CalibrationProfile::DeviceId loadDeviceId(const JsonValue& v)
{
    CalibrationProfile::DeviceId d;
    if (!v.isObject()) return d;
    const auto& obj = v.object;

    getString(obj, "tracking_system", d.trackingSystem);
    getString(obj, "model",  d.model);
    getString(obj, "serial", d.serial);
    return d;
}

Expected<CalibrationProfile> JsonProfileSerializer::deserialize(const std::string& data)
{
    JsonValue v;
    try {
        v = JsonParser(data).parse();
    } catch (const std::exception& e) {
        return Error(ErrorCategory::Configuration, config_error::kParseError, e.what());
    }

    if (!v.isArray())
        return Error(ErrorCategory::Configuration, config_error::kParseError,
                     "Expected JSON array at top level");

    const auto& arr = v.array;
    if (arr.empty())
        return Error(ErrorCategory::Configuration, config_error::kParseError,
                     "Empty profile array");

    if (!arr[0].isObject())
        return Error(ErrorCategory::Configuration, config_error::kParseError,
                     "First profile entry is not an object");

    const auto& obj = arr[0].object;
    CalibrationProfile profile;

    {
        double version = profile.version;
        getNumber(obj, "version", version);
        profile.version = static_cast<uint32_t>(version);
    }
    getString(obj, "name", profile.name);

    // Alignment params
    {
        const auto* alignmentParams = findValue(obj, "alignment_params");
        if (alignmentParams) {
            profile.alignmentParams = loadAlignmentParams(*alignmentParams);
            // continuousCalibrationThreshold lives inside alignment_params in legacy
            if (alignmentParams->isObject()) {
                getNumber(alignmentParams->object,
                          "continuousCalibrationThreshold",
                          profile.continuousCalibrationThreshold);
            }
        }
    }

    getString(obj, "reference_tracking_system", profile.referenceTrackingSystem);
    getString(obj, "target_tracking_system",    profile.targetTrackingSystem);

    double roll = 0, yaw = 0, pitch = 0;
    getNumber(obj, "roll",  roll);
    getNumber(obj, "yaw",   yaw);
    getNumber(obj, "pitch", pitch);
    profile.eulerRotationDegrees = Eigen::Vector3d(roll, yaw, pitch);

    double x = 0, y = 0, z = 0;
    getNumber(obj, "x", x);
    getNumber(obj, "y", y);
    getNumber(obj, "z", z);
    profile.translationCm = Eigen::Vector3d(x, y, z);

    profile.scale = 1.0;
    getNumber(obj, "scale", profile.scale);

    {
        const auto* value = findValue(obj, "reference_device");
        if (value)
            profile.referenceDevice = loadDeviceId(*value);
    }
    {
        const auto* value = findValue(obj, "target_device");
        if (value)
            profile.targetDevice = loadDeviceId(*value);
    }

    profile.autostartContinuous = getBoolean(obj, "autostart_continuous_calibration");
    tryGetBoolean(obj, "enable_static_recalibration", profile.enableStaticRecalibration);
    profile.quashTargetInContinuous = getBoolean(obj, "quash_target_in_continuous");

    {
        double speed = 0;
        getNumber(obj, "calibration_speed", speed);
        profile.speed = static_cast<CalibrationSpeed>(static_cast<int>(speed));
    }

    // Chaperone (optional)
    {
        const auto* chaperoneValue = findValue(obj, "chaperone");
        if (chaperoneValue && chaperoneValue->isObject()) {
            const auto& chObj = chaperoneValue->object;
            platform::ChaperoneData ch;

            tryGetBoolean(chObj, "auto_apply", ch.autoApply);

            if (const auto* playSpaceSize = findValue(chObj, "play_space_size")) {
                loadFloatArray(*playSpaceSize, ch.playSpaceSize, 2);
            }

            if (const auto* standingCenter = findValue(chObj, "standing_center")) {
                loadFloatArray(*standingCenter, ch.standingCenter, 12);
            }

            const auto* geometry = findValue(chObj, "geometry");
            if (geometry && geometry->isArray()) {
                const auto& geo = geometry->array;
                ch.geometryData.resize(geo.size());
                for (size_t i = 0; i < geo.size(); i++)
                    if (geo[i].isNumber())
                        ch.geometryData[i] = static_cast<float>(geo[i].number);
                ch.quadCount = geo.size() / 12;  // 4 vertices * 3 floats each
                ch.valid = !geo.empty();
            }

            if (ch.valid)
                profile.chaperone = ch;
        }
    }

    profile.valid = true;
    return profile;
}

} // namespace spacecal
