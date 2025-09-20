# Docker Setup for Robot Scheduler Demo

This document explains how to run the robot scheduler demo using Docker.

## Prerequisites

- Docker
- Docker Compose (optional, for easier management)

## Quick Start

### 1. Build the Docker image

```bash
make build
# or
docker build -t scheduler-demo .
```

### 2. Run the web interface

```bash
make run
# or
docker-compose up scheduler
```

The web interface will be available at: http://localhost:5000

### 3. Run specific scenarios

```bash
# Run triangle test
make test

# Run specific scenario interactively
make test-scenario
# Then enter scenario name (e.g., triangle_test.txt)
```

## Manual Docker Commands

### Build and run web interface

```bash
docker build -t scheduler-demo .
docker run -p 5000:5000 -v $(pwd)/scenarios:/app/scenarios:ro -v $(pwd)/output:/app/output scheduler-demo
```

### Run scheduler CLI

```bash
# Run triangle test
docker run --rm -v $(pwd)/scenarios:/app/scenarios:ro -v $(pwd)/output:/app/output scheduler-demo python src/scheduler_integrated.py scenarios/triangle_test.txt output/triangle_result.txt

# Run custom scenario
docker run --rm -v $(pwd)/scenarios:/app/scenarios:ro -v $(pwd)/output:/app/output scheduler-demo python src/scheduler_integrated.py scenarios/your_scenario.txt output/your_result.txt
```

## Available Scenarios

The following test scenarios are available in the `scenarios/` directory:

- `triangle_test.txt` - Three robots in triangle formation with central pickup
- `own_base_return_test.txt` - Robots return to their own bases
- `complex_test.txt` - Complex multi-robot coordination
- `warehouse_test.txt` - Warehouse-style scenario
- `multi_level_test.txt` - Multi-level warehouse scenario

## Output

Results are saved to the `output/` directory with the same name as the input scenario but with `_result.txt` suffix.

## Development

For development, you can mount the source code as a volume:

```bash
docker run -p 5000:5000 \
  -v $(pwd)/src:/app/src \
  -v $(pwd)/web:/app/web \
  -v $(pwd)/scenarios:/app/scenarios:ro \
  -v $(pwd)/output:/app/output \
  scheduler-demo
```

## Troubleshooting

### Port already in use
If port 5000 is already in use, change the port mapping:
```bash
docker run -p 5001:5000 scheduler-demo
```

### Permission issues
If you encounter permission issues with output files:
```bash
sudo chown -R $USER:$USER output/
```

### Clean up
To clean up Docker resources:
```bash
make clean
```
