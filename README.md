# CSD2183 Homework 2

This repository contains a C++17 implementation of area-preserving segment
collapse for polygon simplification. The program reads polygon rings from a CSV
file, simplifies them toward a target vertex count, preserves signed area, and
writes the simplified polygon plus summary statistics to standard output.

## Requirements

- `g++` with C++17 support
- `make`
- Windows PowerShell is used by the provided `make test` target

No third-party libraries are required.

## Quick Start

1. **Build** the project:
   ```cmd
   make
   ```

2. **Run tests**:
   ```cmd
   howtotest.bat
   ```

This will build the `simplify` executable and run it against the various test cases, saving the results to `my_output_*.txt` files.

## Usage

Run the program as:

```sh
./simplify <input.csv> <target_vertices>
```

The input CSV must contain the columns:

```text
ring_id,vertex_id,x,y
```

The program writes:

- the simplified polygon rings
- `Total signed area in input`
- `Total signed area in output`
- `Total areal displacement`

## Example Commands

The repository includes several sample inputs in the project root:

```sh
./simplify input_blob_with_two_holes.csv 10 > output_blob.txt
./simplify input_cushion_with_hexagonal_hole.csv 10 > output_cushion.txt
./simplify input_lake_with_two_islands.csv 10 > output_island.txt
./simplify input_rectangle_with_two_holes.csv 10 > output_rectangle.txt
./simplify input_wavy_with_three_holes.csv 10 > output_wavy.txt
```

## Reference Test Cases

Reference test cases are provided in the `test_cases` folder. Each input file
has a matching expected output file.

Official target settings:

| Input file | Target |
|---|---:|
| `input_rectangle_with_two_holes.csv` | 11 |
| `input_cushion_with_hexagonal_hole.csv` | 13 |
| `input_blob_with_two_holes.csv` | 17 |
| `input_wavy_with_three_holes.csv` | 21 |
| `input_lake_with_two_islands.csv` | 17 |
| `input_original_01.csv` | 99 |
| `input_original_02.csv` | 99 |
| `input_original_03.csv` | 99 |
| `input_original_04.csv` | 99 |
| `input_original_05.csv` | 99 |
| `input_original_06.csv` | 99 |
| `input_original_07.csv` | 99 |
| `input_original_08.csv` | 99 |
| `input_original_09.csv` | 99 |
| `input_original_10.csv` | 99 |

For more detail on the provided cases, see:

- `test_cases/README.md`

## Running The Reference Tests

To run the full test suite and compare results against the reference outputs:

```cmd
make test
```

Alternatively, you can run the batch script to generate your own outputs:

```cmd
howtotest.bat
```

## Algorithm Notes

The implementation is based on area-preserving segment collapse:

- candidates are built from 4-vertex windows
- a Steiner point is chosen to preserve area under collapse
- candidates are ranked by areal displacement with local scoring and lookahead
- topology checks reject invalid collapses that would introduce intersections

The main implementation is in `simplify.cpp`.
