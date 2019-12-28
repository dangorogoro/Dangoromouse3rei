/*
 * flash.h
 *
 *  Created on: Oct 26, 2019
 *      Author: dango
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_
extern char _backup_flash_start;

#include "config.h"
#include "Agent.h"
bool Flash_clear();
uint8_t* Flash_load();
bool Flash_store();
void write_mazedata(Maze& maze);
void read_mazedata(Maze& maze);

#endif /* INC_FLASH_H_ */
