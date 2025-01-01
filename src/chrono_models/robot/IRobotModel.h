#ifndef IROBOT_MODEL_H
#define IROBOT_MODEL_H

#include <vector>
#include <memory>
#include "chrono/physics/ChBody.h"

namespace chrono {
namespace models {

class IRobotModel {
  public:
    virtual ~IRobotModel() = default;

    // New method to retrieve collidable bodies with their data file paths
    virtual std::vector<std::pair<std::shared_ptr<chrono::ChBody>, std::string>> GetCollidableBodiesWithPaths()
        const = 0;

    // New method to retrieve visual bodies with their data file paths
    virtual std::vector<std::pair<std::shared_ptr<chrono::ChBody>, std::string>> GetVisualBodiesWithPaths() const = 0;

    // New method to retrieve mesh transforms
    virtual std::vector<std::pair<std::shared_ptr<chrono::ChBody>, ChFrame<>>> GetMeshTransforms() const = 0;

    virtual ChSystem* GetSystem() const = 0;
};

}  // namespace models
}  // namespace chrono

#endif  // IROBOT_MODEL_H