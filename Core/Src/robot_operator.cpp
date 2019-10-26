/*
 * robot_operator.cpp
 *
 *  Created on: Oct 23, 2019
 *      Author: dango
 */

#include "robot_operator.h"
#include "Operation.h"
#include "Maze.h"
xQueueHandle TargetInputQueue = xQueueCreate(1, sizeof(MotionInput));
xQueueHandle RobotStateQueue = xQueueCreate(1, sizeof(RobotState));
xQueueHandle VL6180XDataQueue = xQueueCreate(1, sizeof(VL6180XData));
void RobotOperator::createTask(const char*name, const uint16_t& stack_size,
		const UBaseType_t& task_priority){
	xTaskCreate([](void* obj){
		static_cast<RobotOperator*>(obj)->task();},
		name, stack_size, NULL, task_priority, NULL);
}
void RobotOperator::task(void){
	portTickType xLastWakeTime;
	const uint32_t calledFrequency = 1000;
	const portTickType xFrequency = configTICK_RATE_HZ / calledFrequency;
	xLastWakeTime = xTaskGetTickCount();
	uint16_t count = 0;
	OperationInfo latestOperationInfo(Operation(Operation::FORWARD), NORTH, IndexVec(0, 1));
	//latestOperationInfo.next_dir = NORTH;
	//latestOperationInfo.next_op = Operation(Operation::FORWARD);
	//latestOperationInfo.
	VL6180XData receive_vl6180x_data;
	RobotState lastRobotState(0,0, PI / 2);
	bool turn_state = false;
	int16_t turn_direction = 1;
	MotionInput target_input;
	bool operation_receive_flag = true;
	while(1){
		RobotState receive_robot_state;
		count = (count + 1) % 10000;
		xQueueReceive(RobotStateQueue, &receive_robot_state, 0);

		if(xQueueReceive(VL6180XDataQueue, &receive_vl6180x_data, 0)){
			check_vl6180x(receive_vl6180x_data);
		}
		if(xQueueReceive(OperationInfoQueue, &latestOperationInfo, 0)){
			operation_receive_flag = true;
			turn_state = false;
		}
		if(operation_receive_flag == true && run_search_operation(receive_vl6180x_data, receive_robot_state, latestOperationInfo, target_input, turn_direction, turn_state)){
			Direction wall_info = get_wall_from_ToF(receive_vl6180x_data, latestOperationInfo.next_dir);
			xQueueSendToBack(WallInfoQueue, &wall_info, 0);
			operation_receive_flag = false;
			if(fix_robot_state(receive_vl6180x_data, latestOperationInfo, receive_robot_state)){
				xQueueSendToBack(FixedRobotStateQueue, &receive_robot_state, 0);
			}
		}
		if(operation_receive_flag == false) target_input = MotionInput(0,0);
		xQueueSendToBack(TargetInputQueue, &target_input, 0);
		/*
		if(count % 100 == 0){
			printf("receive state %f, %f, %f \n", receive_robot_state.x,
					receive_robot_state.y, receive_robot_state.theta);
		}
		*/
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
Direction get_wall_from_ToF(const VL6180XData& vl6180x_data, const Direction& dir){
	Direction wall_info;
	bool front_wall, left_wall, right_wall;
	auto Range = vl6180x_data.front_range;
  front_wall = (Range.errorStatus == 0 && Range.range_mm >= 10 && Range.range_mm <= 90)
		? true : false;
  Range = vl6180x_data.left_range;
  left_wall = (Range.errorStatus == 0 && Range.range_mm >= 20 && Range.range_mm <= 70)
		? true : false;
  Range = vl6180x_data.right_range;
  right_wall = (Range.errorStatus == 0 && Range.range_mm >= 20 && Range.range_mm <= 70)
  		? true : false;
	if(dir == NORTH){
		wall_info |= (front_wall) ? NORTH : 0;
		wall_info |= (left_wall) ? WEST : 0;
		wall_info |= (right_wall) ? EAST : 0;
	}
	else if(dir == SOUTH){
		wall_info |= (front_wall) ? SOUTH : 0;
		wall_info |= (left_wall) ? EAST : 0;
		wall_info |= (right_wall) ? WEST : 0;
	}
	else if(dir == EAST){
		wall_info |= (front_wall) ? EAST : 0;
		wall_info |= (left_wall) ? NORTH : 0;
		wall_info |= (right_wall) ? SOUTH : 0;
	}
	else if(dir == WEST){
		wall_info |= (front_wall) ? WEST : 0;
		wall_info |= (left_wall) ? SOUTH : 0;
		wall_info |= (right_wall) ? NORTH : 0;
	}
	return wall_info;
}
bool fix_robot_state(const VL6180XData& vl6180x_data, const OperationInfo& op_info, RobotState& presentRobotState){
	auto nextIndex = op_info.next_index;
	auto nextDirection = op_info.next_dir;
	RobotState targetState(nextIndex.x * MAZE_1BLOCK_LENGTH,
			nextIndex.y * MAZE_1BLOCK_LENGTH, 0);

	auto Range = vl6180x_data.front_range;
  if(Range.errorStatus == 0 && Range.range_mm >= 10 && Range.range_mm <= 100){
  	auto center_value = 35.0;
  	if(nextDirection == NORTH){
  		presentRobotState.y = targetState.y + center_value - Range.range_mm;
  	}
  	else if(nextDirection == SOUTH){
  		presentRobotState.y = targetState.y - center_value + Range.range_mm;
  	}
  	else if(nextDirection == EAST){
  		presentRobotState.x = targetState.x + center_value - Range.range_mm;
  	}
  	else if(nextDirection == WEST){
  		presentRobotState.x = targetState.x - center_value + Range.range_mm;
  	}
  	return true;
  }
  else return false;
}

bool run_search_operation(const VL6180XData& vl6180x_data, RobotState& presentRobotState,
		const OperationInfo& op_info, MotionInput& targetInput, int16_t& turn_direction, bool& turn_state){
	bool ret_flag = false;
	auto latestOP = op_info.next_op;
	auto nextIndex = op_info.next_index;
	auto nextDirection = op_info.next_dir;
	RobotState targetState(nextIndex.x * MAZE_1BLOCK_LENGTH,
			nextIndex.y * MAZE_1BLOCK_LENGTH, 0);
	if(latestOP.op == Operation::FORWARD || turn_state == true){
		RobotState targetState(nextIndex.x * MAZE_1BLOCK_LENGTH, nextIndex.y * MAZE_1BLOCK_LENGTH, 0);
		int32_t length_judge = (nextDirection == NORTH || nextDirection == EAST) ? 1 : -1;
		float compare_length;
		if(nextDirection == NORTH || nextDirection == SOUTH){
			compare_length = targetState.y - presentRobotState.y;
		}
		else{
			compare_length = targetState.x - presentRobotState.x;
		}

		if((compare_length * length_judge) > search_trans_velocity * search_trans_velocity / search_trans_accel / 1000.0){
			targetInput.v = (targetInput.v < search_trans_velocity) ?
					targetInput.v + search_trans_accel : search_trans_velocity;
		}
		else{
			targetInput.v = (targetInput.v > 0.0) ?
					targetInput.v - search_trans_accel : 0.0;
			if(targetInput.v < search_rotate_accel)	ret_flag = true;
		}
		auto target_theta = turn_direction * PI / 2.0;
		targetInput.w = 0 + (target_theta - presentRobotState.theta) * 10;


	}
	else if(latestOP.op == Operation::TURN_LEFT90 || latestOP.op == Operation::TURN_RIGHT90 ||
			latestOP.op == Operation::BACK_180){
		int32_t rad_judge;
		float compare_rad;
		if(latestOP.op == Operation::TURN_LEFT90){
			rad_judge = 1;
			compare_rad = turn_direction * PI / 2.0 + rad_judge * PI / 2.0 - presentRobotState.theta;
		}
		else if(latestOP.op == Operation::TURN_RIGHT90){
			rad_judge = -1;
			compare_rad = turn_direction * PI / 2.0 + rad_judge * PI / 2.0 - presentRobotState.theta;
		}
		else if(latestOP.op == Operation::BACK_180){
			rad_judge = 1;
			compare_rad = turn_direction * PI / 2.0 + 2 * rad_judge * PI / 2.0 - presentRobotState.theta;
		}

		if((rad_judge * compare_rad) > search_rotate_velocity * search_rotate_velocity / search_rotate_accel / 1000.0){
			targetInput.w = (rad_judge * search_rotate_velocity - targetInput.w > 0) ?
					targetInput.w + rad_judge * search_rotate_accel : rad_judge * search_rotate_velocity;
		}
		else{
			targetInput.w = (rad_judge * targetInput.w > 0) ?
					targetInput.w - rad_judge * search_rotate_accel : 0.0;
			if(rad_judge * search_rotate_accel - targetInput.w > 0){
				turn_direction += rad_judge;
				if(latestOP.op == Operation::BACK_180)	turn_direction += rad_judge;
				turn_state = true;
			}
		}
		targetInput.v = 0.0;
	}
	return ret_flag;
}
void check_vl6180x(const VL6180XData& data){
	auto Range = data.front_range;
  if (Range.errorStatus == 0 && Range.range_mm >= 70 && Range.range_mm <= 120){
  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_RESET);
  	printf("FRONT Vaule %d mm\n", (int)Range.range_mm);
  }
  else	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_SET);

  Range = data.left_range;
  if (Range.errorStatus == 0 && Range.range_mm >= 25 && Range.range_mm <= 90){
  	printf("LEFT Vaule %d mm\n", (int)Range.range_mm);
  	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);
  }
  else	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_SET);

  Range = data.right_range;
  if (Range.errorStatus == 0 && Range.range_mm >= 25 && Range.range_mm <= 90){
  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
  	printf("RIGHT Vaule %d mm\n", (int)Range.range_mm);
  }
  else	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
}
