def read_world(filename):
    """
    Read landmark positions from a world definition file.
    
    Args:
        filename: Path to world data file
    
    Returns:
        Dictionary mapping landmark IDs to [x, y] coordinates
    """
    landmarks = {}
    
    try:
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if line:  # Skip empty lines
                    parts = line.split()
                    if len(parts) >= 3:
                        lm_id = int(parts[0])
                        x = float(parts[1])
                        y = float(parts[2])
                        landmarks[lm_id] = [x, y]
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        raise
    except Exception as e:
        print(f"Error reading world file: {e}")
        raise
    
    return landmarks


def read_sensor_data(filename):
    """
    Read odometry and sensor measurements from a data file.
    
    Args:
        filename: Path to sensor data file
    
    Returns:
        Dictionary with structure:
        {
            (timestep, 'odometry'): {'r1': float, 't': float, 'r2': float},
            (timestep, 'sensor'): {'id': list, 'range': list, 'bearing': list}
        }
    """
    sensor_readings = {}
    
    lm_ids = []
    ranges = []
    bearings = []
    
    timestamp = 0
    first_time = True
    
    try:
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:  # Skip empty lines
                    continue
                
                parts = line.split()
                
                if parts[0] == 'ODOMETRY':
                    # Store previous sensor data if not first time
                    if not first_time:
                        sensor_readings[timestamp - 1, 'sensor'] = {
                            'id': lm_ids,
                            'range': ranges,
                            'bearing': bearings
                        }
                        lm_ids = []
                        ranges = []
                        bearings = []
                    else:
                        first_time = False
                    
                    # Store odometry data
                    sensor_readings[timestamp, 'odometry'] = {
                        'r1': float(parts[1]),
                        't': float(parts[2]),
                        'r2': float(parts[3])
                    }
                    
                    timestamp += 1
                
                elif parts[0] == 'SENSOR':
                    # Accumulate sensor readings
                    lm_ids.append(int(parts[1]))
                    ranges.append(float(parts[2]))
                    bearings.append(float(parts[3]))
            
            # Store final sensor data
            if lm_ids:
                sensor_readings[timestamp - 1, 'sensor'] = {
                    'id': lm_ids,
                    'range': ranges,
                    'bearing': bearings
                }
    
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        raise
    except Exception as e:
        print(f"Error reading sensor data file: {e}")
        raise
    
    return sensor_readings