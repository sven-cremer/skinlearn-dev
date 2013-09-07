#include "uta_pr2_forceControl/forceController.h"

namespace pr2_controller_ns{

class PR2NeuroadptControllerClass : public PR2ForceControllerClass
{

public:

  bool init(pr2_mechanism_model::RobotState *robot,
			ros::NodeHandle &n);

  void starting();
  void update();
  void stopping();

  void setFTData();
  void pubFTData();

};
}
