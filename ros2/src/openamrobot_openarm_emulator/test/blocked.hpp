// SPDX-License-Identifier: MIT
#pragma once
#include <gtest/gtest.h>
// A test that cannot run here is BLOCKED. gtest reports it as skipped, never as passed, and CI counts it.
#define BLOCKED(reason) GTEST_SKIP() << "BLOCKED: " << reason
