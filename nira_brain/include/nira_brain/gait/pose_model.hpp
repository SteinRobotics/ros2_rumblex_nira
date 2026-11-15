#pragma once
#include "nira_utils/body_model.hpp"
namespace brain {
using namespace nira_geometry;
class CPoseModel : public nira_geometry::CBodyModel {
   public:
    using CBodyModel::CBodyModel;
};
}  // namespace brain
