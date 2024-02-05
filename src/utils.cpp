#include "mujoco/mujoco.h"
#include <absl/strings/match.h>
#include <iostream>
#include <vector>

// Function to convert enum value to string
const char *enumToString(mjtObj value) {
  switch (value) {
  case mjOBJ_UNKNOWN:
    return "mjOBJ_UNKNOWN";
  case mjOBJ_BODY:
    return "mjOBJ_BODY";
  case mjOBJ_XBODY:
    return "mjOBJ_XBODY";
  case mjOBJ_JOINT:
    return "mjOBJ_JOINT";
  case mjOBJ_DOF:
    return "mjOBJ_DOF";
  case mjOBJ_GEOM:
    return "mjOBJ_GEOM";
  case mjOBJ_SITE:
    return "mjOBJ_SITE";
  case mjOBJ_CAMERA:
    return "mjOBJ_CAMERA";
  case mjOBJ_LIGHT:
    return "mjOBJ_LIGHT";
  case mjOBJ_FLEX:
    return "mjOBJ_FLEX";
  case mjOBJ_MESH:
    return "mjOBJ_MESH";
  case mjOBJ_SKIN:
    return "mjOBJ_SKIN";
  case mjOBJ_HFIELD:
    return "mjOBJ_HFIELD";
  case mjOBJ_TEXTURE:
    return "mjOBJ_TEXTURE";
  case mjOBJ_MATERIAL:
    return "mjOBJ_MATERIAL";
  case mjOBJ_PAIR:
    return "mjOBJ_PAIR";
  case mjOBJ_EXCLUDE:
    return "mjOBJ_EXCLUDE";
  case mjOBJ_EQUALITY:
    return "mjOBJ_EQUALITY";
  case mjOBJ_TENDON:
    return "mjOBJ_TENDON";
  case mjOBJ_ACTUATOR:
    return "mjOBJ_ACTUATOR";
  case mjOBJ_SENSOR:
    return "mjOBJ_SENSOR";
  case mjOBJ_NUMERIC:
    return "mjOBJ_NUMERIC";
  case mjOBJ_TEXT:
    return "mjOBJ_TEXT";
  case mjOBJ_TUPLE:
    return "mjOBJ_TUPLE";
  case mjOBJ_KEY:
    return "mjOBJ_KEY";
  case mjOBJ_PLUGIN:
    return "mjOBJ_PLUGIN";
  default:
    return "Unknown Enum Value";
  }
}

// Print informations relative to the model.
void infos_models(const mjModel *model) {
  std::cout << "\nModel Informations" << std::endl;
  for (int objType = mjOBJ_UNKNOWN; objType < mjOBJ_PLUGIN; ++objType) {
    mjtObj enumValue = static_cast<mjtObj>(objType);

    // Convert enum value to string
    const char *enumName = enumToString(enumValue);

    std::vector<std::pair<const char *, int>> objNames;
    for (int k = 0; k < 100; k++) {
      const char *objName = mj_id2name(model, objType, k);
      if (objName != nullptr) {
        objNames.push_back(std::make_pair(objName, k));
      }
    }
    if (objNames.size() > 0) {
      std::cout << "\n--------- " << enumName + 6 << " ---------" << std::endl;
      for (const auto &element : objNames) {
        const char *objName = element.first;
        int index = element.second;
        std::cout << enumName + 6 << "_" << index << " : " << objName
                  << std::endl;

        // auto it = std::find(foot_names_.begin(), foot_names_.end(),
        //                     std::string(objName));
        // // Create a list of geometry.
        // if (it != foot_names_.end() && objType == mjOBJ_GEOM) {
        //   foot_idx_.push_back(index);
        // }
      }
    }
  }
}

void ParameterIndexes(int indexes[2], const mjModel *model,
                      const std::string_view name) {
  int id =
      // mj_name2id(model, mjOBJ_NUMERIC, absl::StrCat("residual_",
      // name).c_str()); Use residual in name.
      mj_name2id(model, mjOBJ_NUMERIC, std::string(name).c_str());

  if (id == -1) {
    mju_error_s("Parameter '%s' not found", std::string(name).c_str());
  }

  int shift = 0;
  int first_residual = 0;
  int i;
  // Suppose all residual are defined at in block
  for (i = 0; i < model->nnumeric; i++) {
    const char *obj_name = mj_id2name(model, mjOBJ_NUMERIC, i);
    if (i == id) {
      break;
    }
    if (absl::StartsWith(obj_name, "residual_")) {
      shift += model->numeric_size[i];
      first_residual = (first_residual == 0) ? i : first_residual;
    }
  }
  indexes[0] = shift;
  indexes[1] = shift + model->numeric_size[i];
}