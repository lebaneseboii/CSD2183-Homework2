CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic

TARGET := simplify
SRC := simplify.cpp

.PHONY: all clean test

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

test: $(TARGET)
	$(TARGET) ./test_cases/input_rectangle_with_two_holes.csv 7 > simplify_rect.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_rectangle_with_two_holes.txt'; $$b = Get-Content './simplify_rect.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_blob_with_two_holes.csv 17 > simplify_blob.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_blob_with_two_holes.txt'; $$b = Get-Content './simplify_blob.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_original_01.csv 99 > simplify_original_01.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_original_01.txt'; $$b = Get-Content './simplify_original_01.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"

clean:
	del /Q $(TARGET).exe $(TARGET) simplify_rect.out simplify_blob.out simplify_original_01.out 2>nul || exit 0
