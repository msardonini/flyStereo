#pragma once

#include "sqlite3.h"

struct Sqlite3Params {
  sqlite3 *data_base_ = nullptr;
  sqlite3_stmt *sq_stmt = nullptr;
};
