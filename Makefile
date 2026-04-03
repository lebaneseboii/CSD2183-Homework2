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
	$(TARGET) ./test_cases/input_cushion_with_hexagonal_hole.csv 13 > simplify_cushion.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_cushion_with_hexagonal_hole.txt'; $$b = Get-Content './simplify_cushion.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_blob_with_two_holes.csv 17 > simplify_blob.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_blob_with_two_holes.txt'; $$b = Get-Content './simplify_blob.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_wavy_with_three_holes.csv 21 > simplify_wavy.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_wavy_with_three_holes.txt'; $$b = Get-Content './simplify_wavy.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_lake_with_two_islands.csv 17 > simplify_lake.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_lake_with_two_islands.txt'; $$b = Get-Content './simplify_lake.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_original_01.csv 99 > simplify_original_01.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_original_01.txt'; $$b = Get-Content './simplify_original_01.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_original_02.csv 99 > simplify_original_02.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_original_02.txt'; $$b = Get-Content './simplify_original_02.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_original_03.csv 99 > simplify_original_03.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_original_03.txt'; $$b = Get-Content './simplify_original_03.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_original_04.csv 99 > simplify_original_04.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_original_04.txt'; $$b = Get-Content './simplify_original_04.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_original_05.csv 99 > simplify_original_05.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_original_05.txt'; $$b = Get-Content './simplify_original_05.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_original_06.csv 99 > simplify_original_06.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_original_06.txt'; $$b = Get-Content './simplify_original_06.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_original_07.csv 99 > simplify_original_07.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_original_07.txt'; $$b = Get-Content './simplify_original_07.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_original_08.csv 99 > simplify_original_08.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_original_08.txt'; $$b = Get-Content './simplify_original_08.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_original_09.csv 99 > simplify_original_09.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_original_09.txt'; $$b = Get-Content './simplify_original_09.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"
	$(TARGET) ./test_cases/input_original_10.csv 99 > simplify_original_10.out
	powershell -NoProfile -Command "$$a = Get-Content './test_cases/output_original_10.txt'; $$b = Get-Content './simplify_original_10.out'; if ((($$a -join \"`n\") -ne ($$b -join \"`n\"))) { exit 1 }"

clean:
	del /Q $(TARGET).exe $(TARGET) simplify_rect.out simplify_cushion.out simplify_blob.out simplify_wavy.out simplify_lake.out simplify_original_01.out simplify_original_02.out simplify_original_03.out simplify_original_04.out simplify_original_05.out simplify_original_06.out simplify_original_07.out simplify_original_08.out simplify_original_09.out simplify_original_10.out 2>nul || exit 0
