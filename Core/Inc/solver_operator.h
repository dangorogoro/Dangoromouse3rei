/*
 * solver_operator.h
 *
 *  Created on: Oct 23, 2019
 *      Author: dango
 */

#ifndef INC_SOLVER_OPERATOR_H_
#define INC_SOLVER_OPERATOR_H_
#include "config.h"
#include "Agent.h"
#include "flash.h"
#include <memory>

struct OperationInfo{
	Operation next_op;
	Direction next_dir;
	IndexVec next_index;
	OperationInfo(const Operation& _op, const Direction& _dir, const IndexVec& _idx) :
		next_op(_op), next_dir(_dir), next_index(_idx){}
};
class SolverOperator{
private:
	TaskHandle_t handle;
public:
	SolverOperator() : handle(NULL){}
	void createTask(const char* name, const uint16_t& stack_size, const UBaseType_t& task_priority);
	void task();
};
extern Maze &maze;
extern Node &node;
extern Agent &agent;

#endif /* INC_SOLVER_OPERATOR_H_ */
