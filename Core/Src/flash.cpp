#include "flash.h"
#define BACKUP_FLASH_SECTOR_NUM     FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_SIZE    1024*16

// Flashから読みだしたデータを退避するRAM上の領域
// 4byteごとにアクセスをするので、アドレスが4の倍数になるように配置する
static uint8_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__ ((aligned(4)));

// Flashのsector1の先頭に配置される変数(ラベル)
// 配置と定義はリンカスクリプトで行う

void write_mazedata(Maze& maze){
  for(uint8_t y = 0; y < MAZE_SIZE; y++){
    for(uint8_t x = 0; x < MAZE_SIZE; x++){
      work_ram[MAZE_SIZE * y + x] = maze.getWall(x,y);
    }
  }
  Flash_store();
}
void read_mazedata(Maze& maze){
  Flash_load();
  for(uint8_t y = 0; y < MAZE_SIZE; y++){
    for(uint8_t x = 0; x < MAZE_SIZE; x++){
      maze.updateWall(IndexVec(x,y), work_ram[16 * y + x]);
    }
  }
}

// Flashのsector1を消去
bool Flash_clear()
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.NbSectors = 1;

    // Eraseに失敗したsector番号がerror_sectorに入る
    // 正常にEraseができたときは0xFFFFFFFFが入る
    uint32_t error_sector;
    HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

    HAL_FLASH_Lock();

    return result == HAL_OK && error_sector == 0xFFFFFFFF;
}

// Flashのsector1の内容を全てwork_ramに読み出す
// work_ramの先頭アドレスを返す
uint8_t* Flash_load()
{
    memcpy(work_ram, &_backup_flash_start, BACKUP_FLASH_SECTOR_SIZE);
    return work_ram;
}

// Flashのsector1を消去後、work_ramにあるデータを書き込む
bool Flash_store()
{
    // Flashをclear
    if (!Flash_clear()) return false;

    uint32_t *p_work_ram = (uint32_t*)work_ram;

    HAL_FLASH_Unlock();

    // work_ramにあるデータを4バイトごとまとめて書き込む
    HAL_StatusTypeDef result;
    const size_t write_cnt = BACKUP_FLASH_SECTOR_SIZE / sizeof(uint32_t);

    for (size_t i=0; i<write_cnt; i++)
    {
        result = HAL_FLASH_Program(
                    FLASH_TYPEPROGRAM_WORD,
                    (uint32_t)(&_backup_flash_start) + sizeof(uint32_t) * i,
                    p_work_ram[i]
                );
        if (result != HAL_OK) break;
    }

    HAL_FLASH_Lock();

    return result == HAL_OK;
}
