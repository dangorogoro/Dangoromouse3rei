/*
 * solver_operator.cpp
 *
 *  Created on: Oct 23, 2019
 *      Author: dango
 */
#include "solver_operator.h"
#include <memory>

xQueueHandle OperationInfoQueue =  xQueueCreate(1, sizeof(OperationInfo));
xQueueHandle WallInfoQueue = xQueueCreate(1, sizeof(Direction));
const auto p_maze = std::make_unique<Maze>(WallData);
Maze &maze = *p_maze;
const auto p_node = std::make_unique<Node>();
Node &node = *p_node;
const auto p_agent = std::make_unique<Agent>(maze, node);
Agent &agent = *p_agent;

void SolverOperator::createTask(const char*name, const uint16_t& stack_size,
		const UBaseType_t& task_priority){
	xTaskCreate([](void* obj){
		static_cast<SolverOperator*>(obj)->task();},
		name, stack_size, NULL, task_priority, NULL);
}
/*
uint8_t Data21[] = {
  2, 3, 6, 10, 10, 10, 2, 10, 10, 10, 10, 3, 6, 3, 6, 3,
  1, 5, 5, 6, 3, 6, 9, 6, 10, 10, 10, 9, 5, 5, 5, 5,
  1, 12, 9, 5, 4, 8, 11, 12, 10, 10, 10, 3, 5, 12, 9, 5,
  0, 10, 10, 9, 5, 6, 10, 10, 10, 10, 10, 1, 12, 10, 3, 5,
  1, 6, 10, 2, 9, 5, 6, 2, 10, 3, 6, 9, 6, 10, 1, 5,
  0, 9, 6, 9, 7, 5, 5, 4, 3, 5, 12, 3, 5, 6, 9, 5,
  1, 6, 8, 10, 1, 5, 4, 1, 13, 5, 6, 9, 4, 8, 3, 5,
  0, 9, 6, 10, 1, 5, 5, 4, 3, 5, 5, 6, 8, 2, 9, 5,
  0, 2, 8, 3, 5, 5, 5, 4, 9, 5, 4, 9, 6, 8, 11, 5,
  1, 12, 3, 12, 1, 5, 5, 4, 3, 12, 9, 6, 8, 11, 6, 1,
  8, 10, 8, 10, 1, 4, 1, 5, 5, 6, 10, 1, 14, 2, 1, 5,
  2, 10, 10, 10, 1, 5, 5, 4, 1, 4, 10, 9, 6, 9, 5, 5,
  8, 10, 3, 14, 1, 12, 8, 1, 12, 1, 6, 3, 5, 6, 9, 5,
  2, 3, 4, 3, 4, 10, 3, 12, 10, 9, 5, 5, 5, 4, 10, 9,
  1, 5, 5, 5, 5, 14, 8, 10, 10, 10, 9, 12, 9, 12, 10, 3,
  9, 12, 9, 12, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9
};
void SolverOperator::task(){
	portTickType xLastWakeTime;
	const uint32_t calledFrequency = 100;
	const portTickType xFrequency = configTICK_RATE_HZ / calledFrequency;
	xLastWakeTime = xTaskGetTickCount();

  auto sampleData = Data21;
  IndexVec po(0,1);
  Direction dir;
  dir = sampleData[0 + 16 * (15 - 1)];

  bool flag = false;
	while(1){
		if(flag == false){
		dir = sampleData[po.x + 16 *(15 - po.y)];
		agent.update(po, dir.byte | 0xf0);
		po = agent.getNextIndex();
		}
		if(flag == false && agent.getState() == Agent::FINISHED){
			//maze.printWall(ans_path);
		  node.startFastestMap(0, GOAL, true);
		  auto ans_path = node.getPathQueue(0, GOAL);
		  HAL_Delay(100);
		  maze.printWall(ans_path);
		  HAL_Delay(100);
		  maze.printWall(node);

			flag = true;
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
*/

void SolverOperator::task(){
	portTickType xLastWakeTime;
	const uint32_t calledFrequency = 100;
	const portTickType xFrequency = configTICK_RATE_HZ / calledFrequency;
	xLastWakeTime = xTaskGetTickCount();
  OperationInfo latestOPInfo(Operation(Operation::FORWARD), NORTH, IndexVec(0,1));
	xQueueSendToBack(OperationInfoQueue, &latestOPInfo, 0);
  Direction wallInfo;
	while(1){
		if(xQueueReceive(WallInfoQueue, &wallInfo, 0)){
			agent.update(latestOPInfo.next_index, wallInfo.byte | 0xf0);
			latestOPInfo.next_dir = agent.getNextDirection();
			latestOPInfo.next_index = agent.getNextIndex();
			latestOPInfo.next_op = agent.getNextOperation();
			xQueueSendToBack(OperationInfoQueue, &latestOPInfo, 0);
		}
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

