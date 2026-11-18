#pragma once

#include "robot_sim/simulation.hpp"
#include <filesystem>
#include <memory>
#include <span>
#include <string_view>

int run_scenario(std::string_view command, std::span<char*> arguments,
                 std::shared_ptr<const robot_sim::Module> module, uint64_t module_hash);

