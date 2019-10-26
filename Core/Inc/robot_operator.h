/*
 * robot_operator.h
 *
 *  Created on: Oct 23, 2019
 *      Author: dango
 */

#ifndef INC_ROBOT_OPERATOR_H_
#define INC_ROBOT_OPERATOR_H_

#include "config.h"
#include "motion_observer.h"
#include "my_vl6180x.h"
#include "solver_operator.h"
class RobotOperator{
private:
	TaskHandle_t handle;
public:
	RobotOperator() : handle(NULL) {}
	void createTask(const char* name, const uint16_t& stack_size, const UBaseType_t& task_priority);
	void task();
};
Direction get_wall_from_ToF(const VL6180XData& vl6180x_data, const Direction& dir);
bool fix_robot_state(const VL6180XData& vl6180x_data, const OperationInfo& op_info, RobotState& presentRobotState);
bool run_search_operation(const VL6180XData& vl6180x_data, RobotState& presentRobotState,
		const OperationInfo& op_info, MotionInput& targetInput, int16_t& turn_direction, bool& turn_state);

/*
bool run_search_operation(const RobotState& presentRobotState,
		const OperationInfo& op_info, MotionInput& targetInput, RobotState& lastRobotState, int16_t& turn_direction, bool& turn_state);
*/
void check_vl6180x(const VL6180XData& data);
#endif /* INC_ROBOT_OPERATOR_H_ */
