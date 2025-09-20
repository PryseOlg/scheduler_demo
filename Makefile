# Makefile for Docker operations

.PHONY: build run stop clean test help

# Build the Docker image
build:
	docker build -t scheduler-demo .

# Run the web interface
run:
	docker-compose up scheduler

# Run scheduler CLI with triangle test (for testing only)
test:
	docker-compose run --rm scheduler-cli

# Stop all containers
stop:
	docker-compose down

# Clean up containers and images
clean:
	docker-compose down --rmi all --volumes --remove-orphans
	docker system prune -f

# Show help
help:
	@echo "Available commands:"
	@echo "  build         - Build Docker image"
	@echo "  run           - Run web interface"
	@echo "  test          - Run triangle test scenario (for testing only)"
	@echo "  stop          - Stop all containers"
	@echo "  clean         - Clean up Docker resources"
	@echo "  help          - Show this help"
