#include <spacecal/app/profile_manager.h>
#include <spacecal/core/calibration_policy.h>

#include <picojson.h>

#include <sstream>

namespace spacecal {

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

static picojson::object serializeAlignmentParams(const AlignmentSpeedParams& p)
{
    picojson::object obj;
    obj["align_speed_tiny"]  = picojson::value(p.align_speed_tiny);
    obj["align_speed_small"] = picojson::value(p.align_speed_small);
    obj["align_speed_large"] = picojson::value(p.align_speed_large);
    obj["thr_trans_tiny"]    = picojson::value(p.thr_trans_tiny);
    obj["thr_trans_small"]   = picojson::value(p.thr_trans_small);
    obj["thr_trans_large"]   = picojson::value(p.thr_trans_large);
    obj["thr_rot_tiny"]      = picojson::value(p.thr_rot_tiny);
    obj["thr_rot_small"]     = picojson::value(p.thr_rot_small);
    obj["thr_rot_large"]     = picojson::value(p.thr_rot_large);
    return obj;
}

static picojson::object serializeDeviceId(const CalibrationProfile::DeviceId& d)
{
    picojson::object obj;
    obj["tracking_system"] = picojson::value(d.trackingSystem);
    obj["model"] = picojson::value(d.model);
    obj["serial"] = picojson::value(d.serial);
    return obj;
}

static picojson::array floatArrayToJson(const float* buf, int count)
{
    picojson::array arr;
    arr.reserve(count);
    for (int i = 0; i < count; i++)
        arr.push_back(picojson::value(static_cast<double>(buf[i])));
    return arr;
}

static void loadFloatArray(const picojson::value& v, float* buf, int count)
{
    if (!v.is<picojson::array>()) return;
    const auto& arr = v.get<picojson::array>();
    for (int i = 0; i < count && i < static_cast<int>(arr.size()); i++)
        buf[i] = static_cast<float>(arr[i].get<double>());
}

// ============================================================================
// JsonProfileSerializer::serialize
// ============================================================================

std::string JsonProfileSerializer::serialize(const CalibrationProfile& profile)
{
    picojson::object obj;

    obj["version"] = picojson::value(static_cast<double>(profile.version));
    obj["name"] = picojson::value(profile.name);
    obj["alignment_params"] = picojson::value(
        serializeAlignmentParams(profile.alignmentParams));

    // Continuous calibration threshold is stored alongside alignment params
    // in the legacy format. Mirror that behavior.
    {
        auto& ap = obj["alignment_params"].get<picojson::object>();
        ap["continuousCalibrationThreshold"] =
            picojson::value(profile.continuousCalibrationThreshold);
    }

    obj["reference_tracking_system"] = picojson::value(profile.referenceTrackingSystem);
    obj["target_tracking_system"] = picojson::value(profile.targetTrackingSystem);

    // eulerRotationDegrees: (0)=roll, (1)=yaw, (2)=pitch
    obj["roll"] = picojson::value(profile.eulerRotationDegrees(0));
    obj["yaw"] = picojson::value(profile.eulerRotationDegrees(1));
    obj["pitch"] = picojson::value(profile.eulerRotationDegrees(2));

    // translationCm: (0)=x, (1)=y, (2)=z
    obj["x"] = picojson::value(profile.translationCm(0));
    obj["y"] = picojson::value(profile.translationCm(1));
    obj["z"] = picojson::value(profile.translationCm(2));

    obj["scale"] = picojson::value(profile.scale);

    obj["reference_device"] = picojson::value(
        serializeDeviceId(profile.referenceDevice));
    obj["target_device"] = picojson::value(
        serializeDeviceId(profile.targetDevice));

    obj["autostart_continuous_calibration"] = picojson::value(profile.autostartContinuous);
    obj["enable_static_recalibration"] = picojson::value(profile.enableStaticRecalibration);
    obj["quash_target_in_continuous"] = picojson::value(profile.quashTargetInContinuous);
    obj["calibration_speed"] = picojson::value(
        static_cast<double>(static_cast<int>(profile.speed)));

    if (profile.chaperone.has_value()) {
        const auto& ch = profile.chaperone.value();
        picojson::object chObj;
        chObj["auto_apply"] = picojson::value(ch.autoApply);
        chObj["play_space_size"] = picojson::value(
            floatArrayToJson(ch.playSpaceSize, 2));
        chObj["standing_center"] = picojson::value(
            floatArrayToJson(ch.standingCenter, 12));
        chObj["geometry"] = picojson::value(
            floatArrayToJson(ch.geometryData.data(),
                             static_cast<int>(ch.geometryData.size())));
        obj["chaperone"] = picojson::value(chObj);
    }

    // Wrap in array (legacy format compatibility)
    picojson::value profileV(obj);

    picojson::array profiles;
    profiles.push_back(profileV);

    picojson::value profilesV(profiles);

    return profilesV.serialize(true);
}

// ============================================================================
// JsonProfileSerializer::deserialize
// ============================================================================

static AlignmentSpeedParams loadAlignmentParams(const picojson::value& v)
{
    AlignmentSpeedParams p;
    if (!v.is<picojson::object>()) return p;
    const auto& obj = v.get<picojson::object>();

    auto get = [&](const char* key, double& out) {
        auto it = obj.find(key);
        if (it != obj.end() && it->second.is<double>())
            out = it->second.get<double>();
    };
    get("align_speed_tiny",  p.align_speed_tiny);
    get("align_speed_small", p.align_speed_small);
    get("align_speed_large", p.align_speed_large);
    get("thr_trans_tiny",    p.thr_trans_tiny);
    get("thr_trans_small",   p.thr_trans_small);
    get("thr_trans_large",   p.thr_trans_large);
    get("thr_rot_tiny",      p.thr_rot_tiny);
    get("thr_rot_small",     p.thr_rot_small);
    get("thr_rot_large",     p.thr_rot_large);
    return p;
}

static CalibrationProfile::DeviceId loadDeviceId(const picojson::value& v)
{
    CalibrationProfile::DeviceId d;
    if (!v.is<picojson::object>()) return d;
    const auto& obj = v.get<picojson::object>();

    auto getStr = [&](const char* key, std::string& out) {
        auto it = obj.find(key);
        if (it != obj.end() && it->second.is<std::string>())
            out = it->second.get<std::string>();
    };
    getStr("tracking_system", d.trackingSystem);
    getStr("model",  d.model);
    getStr("serial", d.serial);
    return d;
}

Expected<CalibrationProfile> JsonProfileSerializer::deserialize(const std::string& data)
{
    picojson::value v;
    std::string err = picojson::parse(v, data);
    if (!err.empty())
        return Error(ErrorCategory::Configuration, config_error::kParseError, err);

    if (!v.is<picojson::array>())
        return Error(ErrorCategory::Configuration, config_error::kParseError,
                     "Expected JSON array at top level");

    const auto& arr = v.get<picojson::array>();
    if (arr.empty())
        return Error(ErrorCategory::Configuration, config_error::kParseError,
                     "Empty profile array");

    if (!arr[0].is<picojson::object>())
        return Error(ErrorCategory::Configuration, config_error::kParseError,
                     "First profile entry is not an object");

    const auto& obj = arr[0].get<picojson::object>();
    CalibrationProfile profile;

    auto getStr = [&](const char* key, std::string& out) {
        auto it = obj.find(key);
        if (it != obj.end() && it->second.is<std::string>())
            out = it->second.get<std::string>();
    };
    auto getDbl = [&](const char* key, double& out) {
        auto it = obj.find(key);
        if (it != obj.end() && it->second.is<double>())
            out = it->second.get<double>();
    };
    auto getBool = [&](const char* key) -> bool {
        auto it = obj.find(key);
        if (it != obj.end()) return it->second.evaluate_as_boolean();
        return false;
    };
    auto tryGetBool = [&](const char* key, bool& out) {
        auto it = obj.find(key);
        if (it != obj.end()) {
            out = it->second.evaluate_as_boolean();
        }
    };

    {
        double version = profile.version;
        getDbl("version", version);
        profile.version = static_cast<uint32_t>(version);
    }
    getStr("name", profile.name);

    // Alignment params
    {
        auto it = obj.find("alignment_params");
        if (it != obj.end()) {
            profile.alignmentParams = loadAlignmentParams(it->second);
            // continuousCalibrationThreshold lives inside alignment_params in legacy
            if (it->second.is<picojson::object>()) {
                auto jt = it->second.get<picojson::object>().find(
                    "continuousCalibrationThreshold");
                if (jt != it->second.get<picojson::object>().end() &&
                    jt->second.is<double>())
                    profile.continuousCalibrationThreshold = jt->second.get<double>();
            }
        }
    }

    getStr("reference_tracking_system", profile.referenceTrackingSystem);
    getStr("target_tracking_system",    profile.targetTrackingSystem);

    double roll = 0, yaw = 0, pitch = 0;
    getDbl("roll",  roll);
    getDbl("yaw",   yaw);
    getDbl("pitch", pitch);
    profile.eulerRotationDegrees = Eigen::Vector3d(roll, yaw, pitch);

    double x = 0, y = 0, z = 0;
    getDbl("x", x); getDbl("y", y); getDbl("z", z);
    profile.translationCm = Eigen::Vector3d(x, y, z);

    profile.scale = 1.0;
    getDbl("scale", profile.scale);

    {
        auto it = obj.find("reference_device");
        if (it != obj.end())
            profile.referenceDevice = loadDeviceId(it->second);
    }
    {
        auto it = obj.find("target_device");
        if (it != obj.end())
            profile.targetDevice = loadDeviceId(it->second);
    }

    profile.autostartContinuous    = getBool("autostart_continuous_calibration");
    tryGetBool("enable_static_recalibration", profile.enableStaticRecalibration);
    profile.quashTargetInContinuous = getBool("quash_target_in_continuous");

    {
        double speed = 0;
        getDbl("calibration_speed", speed);
        profile.speed = static_cast<CalibrationSpeed>(static_cast<int>(speed));
    }

    // Chaperone (optional)
    {
        auto it = obj.find("chaperone");
        if (it != obj.end() && it->second.is<picojson::object>()) {
            const auto& chObj = it->second.get<picojson::object>();
            platform::ChaperoneData ch;

            auto bit = chObj.find("auto_apply");
            if (bit != chObj.end()) ch.autoApply = bit->second.evaluate_as_boolean();

            auto psit = chObj.find("play_space_size");
            if (psit != chObj.end())
                loadFloatArray(psit->second, ch.playSpaceSize, 2);

            auto scit = chObj.find("standing_center");
            if (scit != chObj.end())
                loadFloatArray(scit->second, ch.standingCenter, 12);

            auto git = chObj.find("geometry");
            if (git != chObj.end() && git->second.is<picojson::array>()) {
                const auto& geo = git->second.get<picojson::array>();
                ch.geometryData.resize(geo.size());
                for (size_t i = 0; i < geo.size(); i++)
                    ch.geometryData[i] = static_cast<float>(geo[i].get<double>());
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
