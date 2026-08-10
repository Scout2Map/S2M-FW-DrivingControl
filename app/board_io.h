/*
 * File   : board_io.h
 * Purpose: Exposes the concrete drive_io_t table for this board.
 * Author : jihoonkimtech
 */

#ifndef BOARD_IO_H
#define BOARD_IO_H

#include "drive.h"

// Returns the concrete IO table wiring the control layer to this board
const drive_io_t *board_io_get(void);

#endif
