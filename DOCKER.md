# Docker Commands for OMPL-SMR Project

## Build Docker Image

```powershell
cd "e:\COMP 550\Project - G"
docker build -t ompl-smr .
```

## Run Docker Container

### Interactive Mode (for development)
```powershell
docker run -it --rm -v "${PWD}:/workspace" ompl-smr
```

### Run with Graphics (if using X11)
```powershell
docker run -it --rm -v "${PWD}:/workspace" -e DISPLAY=host.docker.internal:0 ompl-smr
```

## Inside Docker Container

### Setup Environments
```bash
python3 main.py --setup
```

### Run Part 1 with OMPL
```bash
python3 part1_ompl.py --env environments/simple_env.json
```

### Run Part 1 with Pure Python (no OMPL)
```bash
python3 part1_ompl.py --env environments/simple_env.json --force-python
```

### Run Sensitivity Analysis
```bash
python3 part1_ompl.py --sensitivity --n_states 1500
```

### Run Part 2
```bash
python3 part2_ompl.py --env environments/simple_env.json --n_trials 20
```

## Docker Compose (Alternative)

Create `docker-compose.yml`:

```yaml
version: '3'
services:
  ompl-smr:
    build: .
    volumes:
      - .:/workspace
    environment:
      - PYTHONUNBUFFERED=1
    stdin_open: true
    tty: true
```

Then run:
```powershell
docker-compose up -d
docker-compose exec ompl-smr bash
```

## Verify OMPL Installation

Inside container:
```bash
python3 -c "from ompl import base as ob; print('OMPL version:', ob.OMPL_VERSION)"
```

## Save Results from Docker

Results are automatically saved to the mounted `/workspace` directory,
which maps to your local project folder. All files in `results/` will
be accessible on your host machine.

## Useful Docker Commands

```powershell
# List running containers
docker ps

# Stop container
docker stop <container_id>

# Remove container
docker rm <container_id>

# Remove image
docker rmi ompl-smr

# Clean up unused images
docker system prune
```

## Troubleshooting

### Issue: OMPL not found
```bash
# Inside container, check if OMPL is installed
ls /usr/local/lib | grep ompl
ldconfig
python3 -c "import sys; print(sys.path)"
```

### Issue: Graphics not showing
Use headless mode or save figures to file:
```bash
python3 part1_ompl.py --env environments/simple_env.json
# Figures will be saved to results/ directory
```

### Issue: Permission errors
```powershell
# On Windows, ensure Docker has access to the drive
# Docker Desktop -> Settings -> Resources -> File Sharing
```
