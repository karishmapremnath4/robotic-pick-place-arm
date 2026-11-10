CXX      := clang++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wpedantic -Iinclude
BUILD    := build

# Firmware. These files must also cross compile for the target, so nothing
# here may use the heap, exceptions, or host only libraries.
FW_SRC := src/drivers/imu.cpp src/drivers/encoder.cpp src/drivers/can_node.cpp \
          src/drivers/laser.cpp \
          src/est/ekf.cpp src/diag/sensor_health.cpp src/diag/safety.cpp \
          src/task/sequence.cpp src/task/machine.cpp \
          src/task/survey.cpp src/task/planner.cpp \
          src/rtos/scheduler.cpp

# Simulation. The test bench: host only, never cross compiled.
SIM_SRC := src/sim/arm_dynamics.cpp src/sim/motor.cpp src/sim/plant.cpp \
           src/sim/kinematics.cpp src/sim/trajectory.cpp src/sim/world.cpp \
           src/sim/sensor_board.cpp

SRC := $(FW_SRC) $(SIM_SRC)
OBJ := $(SRC:src/%.cpp=$(BUILD)/%.o)

APPS := $(BUILD)/ppa $(BUILD)/record $(BUILD)/plant_check $(BUILD)/sensor_check $(BUILD)/ctrl_check $(BUILD)/est_check $(BUILD)/fault_check $(BUILD)/task_check $(BUILD)/survey_check $(BUILD)/plan_check

all: $(APPS)

$(BUILD)/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: app/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/ppa: $(OBJ) $(BUILD)/ppa.o
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/record: $(OBJ) $(BUILD)/record.o
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/plant_check: $(OBJ) $(BUILD)/plant_check.o
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/sensor_check: $(OBJ) $(BUILD)/sensor_check.o
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/ctrl_check: $(OBJ) $(BUILD)/ctrl_check.o
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/est_check: $(OBJ) $(BUILD)/est_check.o
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/fault_check: $(OBJ) $(BUILD)/fault_check.o
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/task_check: $(OBJ) $(BUILD)/task_check.o
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/survey_check: $(OBJ) $(BUILD)/survey_check.o
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/plan_check: $(OBJ) $(BUILD)/plan_check.o
	$(CXX) $(CXXFLAGS) $^ -o $@

CHECKS := plant_check sensor_check ctrl_check est_check fault_check \
          task_check survey_check plan_check

# Run every check suite and summarise. Non zero exit if anything failed.
test: $(addprefix $(BUILD)/,$(CHECKS))
	@fail=0; total=0; \
	for t in $(CHECKS); do \
	  n=$$(./$(BUILD)/$$t 2>&1 | grep -cE '^  .* (PASS|FAIL) '); \
	  if ./$(BUILD)/$$t > /dev/null 2>&1; then r="ok  "; else r="FAIL"; fail=1; fi; \
	  total=$$((total+n)); \
	  printf "  %-14s %s  %3s checks\n" "$$t" "$$r" "$$n"; \
	done; \
	echo; printf "  %d checks total\n" "$$total"; \
	if [ $$fail -ne 0 ]; then echo "  SUITES FAILED"; exit 1; fi; \
	echo "  all suites pass"

# Figures from the last recorded run.
plots:
	python3 tools/plots.py

# Record a scripted run and build the self contained viewer page.
viewer: $(BUILD)/record
	@mkdir -p data
	./$(BUILD)/record
	python3 tools/make_viewer.py
	@echo
	@echo "open data/viewer.html in a browser"

clean:
	rm -rf $(BUILD)

-include $(shell find $(BUILD) -name '*.d' 2>/dev/null)

.PHONY: all clean viewer test plots
