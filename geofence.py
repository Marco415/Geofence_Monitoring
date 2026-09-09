import math
import time
import json
import logging
from gpiozero import Servo  # type: ignore

# ================================================
# CONFIGURATION
# ================================================
# -----Circular geofence-----
# Radius of circular geofence in metres
RADIUS_METERS = 1000

# -----Polygon geofence-----
POLYGON = [
    (-25.7475, 28.2288),
    (-25.7470, 28.2300),
    (-25.7482, 28.2310),
    (-25.7490, 28.2298),
    (-25.7488, 28.2285)
]

# -----Elliptical geofence-----
# Centre of geofence
CENTER_LAT = -26.72
CENTER_LON = 27.09
# Semi-major axis in metres
MAJOR_AXIS = 50
# Semi-minor axis in metres
MINOR_AXIS = 40
# Rotation of ellipse clockwise from North
ROTATION = 30

# -----Geofence type-----
GEOFENCE_TYPE = 3

# -----Monitoring-----
# Time between geofence checks (seconds)
CHECK_INTERVAL = 5

# -----GPS coordinate limits-----
NORTH_LIMIT = -22
EAST_LIMIT = 33
SOUTH_LIMIT = -35
WEST_LIMIT = 16

# -----GPS configuration-----
LATEST_GPS_FILE = f"GroundTest/latest_gps.json"

# GPS stability configuration
# Number of GPS readings used for smoothing.
# A higher value gives more stability but makes the system slower to react.
GPS_SMOOTHING_SAMPLES = 5

# Number of consecutive OUTSIDE readings required before activating the servo.
# This prevents one noisy GPS reading from immediately triggering the system.
OUTSIDE_CONFIRMATIONS_REQUIRED = 3

# Delay between GPS file read attempts  when invalid data is encountered.
GPS_RETRY_DELAY = 1

# Number of attempts to get a valid GPS reading during each monitoring cycle.
GPS_READ_ATTEMPTS = 5

# -----Servo configuration-----
SERVO_PIN = 18
# Servo idle position
NORMAL_POSITION = -1.0
# Servo activated position
ACTIVATE_POSITION = 1.0
# Time allowed for servo to physically move
MOVE_TIME = 1.0
# How long the servo stays activated
ACTIVATE_TIME = 5.0
# Return to idle after activation
RETURN_TO_NORMAL = True

# ================================================
# LOGGING
# ================================================
LOG_FILE = "geofence.log"

logging.basicConfig(
    filename=LOG_FILE,
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    force=True
)


# ================================================
# GLOBAL VARIABLES
# ================================================
# Last valid GPS position
last_valid_latitude = CENTER_LAT
last_valid_longitude = CENTER_LON
# GPS history used for smoothing
gps_history = []
# Number of consecutive outside detections
outside_count = 0
# Prevents the servo from being activated more than once
servo_triggered = False

# ================================================
# SERVO SETUP
# ================================================
servo = None
def initialise_servo():
    global servo

    try:
        print("Initialising servo...")
        servo = Servo(SERVO_PIN)

        # Set servo to idle position once.
        servo.value = NORMAL_POSITION
        print( f"Servo initialised on GPIO {SERVO_PIN}. Idle position = {NORMAL_POSITION}")

        logging.info(
            f"Servo initialised on GPIO {SERVO_PIN}. "
            f"Idle position = {NORMAL_POSITION}"
        )

        # Give the servo time to physically reach its idle position.
        time.sleep(MOVE_TIME)
        print("Servo is now IDLE and ready.")
        print()

    except Exception as error:
        print(f"ERROR: Could not initialise servo: {error}")
        logging.exception(
            f"Servo initialisation failed: {error}"
        )
        raise

# ================================================
# GPS VALIDATION
# ================================================
def validate_coordinates(latitude, longitude):
    if latitude is None or longitude is None:
        return False

    try:
        latitude = float(latitude)
        longitude = float(longitude)

    except (TypeError, ValueError):
        return False

    # Latitude limits
    if latitude < SOUTH_LIMIT or latitude > NORTH_LIMIT:
        return False

    # Longitude limits
    if longitude < WEST_LIMIT or longitude > EAST_LIMIT:
        return False

    return True

# ================================================
# READ GPS FILE
# ================================================
def read_latest_gps():
    try:
        with open(LATEST_GPS_FILE, "r") as file:
            data = json.load(file)

        if not isinstance(data, dict):
            message = ("GPS file does not contain a JSON object.")
            print(f"ERROR: {message}")
            logging.error(message)
            return None

        return data

    except FileNotFoundError:
        message = (f"GPS file not found: {LATEST_GPS_FILE}")
        print(f"ERROR: {message}")
        logging.error(message)
        return None

    except json.JSONDecodeError as error:
        message = ( f"GPS JSON could not be parsed: {error}")
        print(f"ERROR: {message}")
        logging.error(message)
        return None

    except PermissionError:
        message = (f"Permission denied when reading GPS file: {LATEST_GPS_FILE}")
        print(f"ERROR: {message}")
        logging.error(message)
        return None

    except OSError as error:
        message = (f"GPS file read error: {error}")
        print(f"ERROR: {message}")
        logging.error(message)
        return None

    except Exception as error:
        message = (f"Unexpected GPS read error: {error}")
        print(f"ERROR: {message}")
        logging.exception(message)
        return None

# ================================================
# GPS SMOOTHING
# ================================================
def median(values):
    if not values:
        return None

    sorted_values = sorted(values)
    middle = len(sorted_values) // 2

    if len(sorted_values) % 2 == 0:
        return (
            sorted_values[middle - 1]
            + sorted_values[middle]
        ) / 2

    return sorted_values[middle]

def smooth_coordinates(latitude, longitude):
    global gps_history
    gps_history.append((latitude, longitude))

    # Keep only the required number of readings.
    if len(gps_history) > GPS_SMOOTHING_SAMPLES:
        gps_history.pop(0)

    latitudes = [
        position[0]
        for position in gps_history
    ]

    longitudes = [
        position[1]
        for position in gps_history
    ]

    smoothed_latitude = median(latitudes)
    smoothed_longitude = median(longitudes)

    return (smoothed_latitude, smoothed_longitude)

# ================================================
# GET CURRENT GPS LOCATION
# ================================================
def get_current_location():
    global last_valid_latitude
    global last_valid_longitude

    for attempt in range(1, GPS_READ_ATTEMPTS + 1):
        data = read_latest_gps()

        if data is None:
            print(f"GPS read attempt {attempt}/{GPS_READ_ATTEMPTS} failed.")
            logging.warning(
                f"GPS read attempt "
                f"{attempt}/{GPS_READ_ATTEMPTS} failed."
            )
            time.sleep(GPS_RETRY_DELAY)
            continue

        # Extract GPS values
        latitude = data.get("latitude")
        longitude = data.get("longitude")

        altitude = data.get("altitude_m")
        speed = data.get("speed_mps")
        track = data.get("track_deg")
        satellites = data.get("satellites_used")

        # Validate coordinates
        if validate_coordinates(latitude, longitude):
            latitude = float(latitude)
            longitude = float(longitude)

            # Add reading to smoothing filter
            smoothed_latitude, smoothed_longitude = (
                smooth_coordinates(latitude, longitude)
            )

            # Save last valid position
            last_valid_latitude = smoothed_latitude
            last_valid_longitude = smoothed_longitude

            print(f"GPS reading: raw=({latitude:.6f}, {longitude:.6f})")
            print(f"Smoothed position: ({smoothed_latitude:.6f}, {smoothed_longitude:.6f})")
            print(f"GPS information: altitude={altitude} m, speed={speed} m/s, track={track}°, satellites={satellites}")

            logging.info(
                f"GPS | "
                f"Raw=({latitude:.6f}, {longitude:.6f}) | "
                f"Smoothed=({smoothed_latitude:.6f}, "
                f"{smoothed_longitude:.6f}) | "
                f"Altitude={altitude}m | "
                f"Speed={speed}m/s | "
                f"Track={track}° | "
                f"Satellites={satellites}"
            )

            return (smoothed_latitude, smoothed_longitude)

        else:
            print(f"WARNING: Invalid GPS coordinates received: ({latitude}, {longitude})")
            logging.warning(
                f"Invalid GPS coordinates: "
                f"({latitude}, {longitude})"
            )
            time.sleep(GPS_RETRY_DELAY)

    # No valid GPS data was obtained.
    # Continue using the last known valid coordinate rather than creating a false outside-geofence trigger.
    print(f"WARNING: No valid GPS reading received after {GPS_READ_ATTEMPTS} attempts.")
    print(f"Using last valid position: ({last_valid_latitude:.6f}, {last_valid_longitude:.6f})")
    logging.warning(
        f"No valid GPS reading after "
        f"{GPS_READ_ATTEMPTS} attempts. "
        f"Using last valid position: "
        f"({last_valid_latitude:.6f}, "
        f"{last_valid_longitude:.6f})"
    )

    return (last_valid_latitude, last_valid_longitude)

# ================================================
# CIRCULAR GEOFENCE
# ================================================
def haversine(lat1, lon1, lat2, lon2):
    earth_radius = 6371000

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)

    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = (
        math.sin(dphi / 2) ** 2 +
        math.cos(phi1) *
        math.cos(phi2) *
        math.sin(dlambda / 2) ** 2
    )

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return earth_radius * c


# ================================================
# POLYGON GEOFENCE
# ================================================
def point_inside_polygon(lat, lon, polygon):
    inside = False
    n = len(polygon)
    j = n - 1

    for i in range(n):
        yi, xi = polygon[i]
        yj, xj = polygon[j]

        intersect = (
            ((xi > lon) != (xj > lon))
            and
            (lat < (yj - yi) * (lon - xi) / (xj - xi + 1e-12) + yi)
        )

        if intersect:
            inside = not inside

        j = i

    return inside


# ================================================
# ELLIPTICAL GEOFENCE
# ================================================
def gps_to_local(lat, lon, center_lat, center_lon):
    earth_radius = 6371000
    dlat = math.radians(lat - center_lat)
    dlon = math.radians(lon - center_lon)

    x = (dlon * earth_radius * math.cos(math.radians(center_lat)))
    y = (dlat * earth_radius)

    return x, y

def rotate_point(x, y, angle_deg):
    theta = math.radians(angle_deg)
    xr = (x * math.cos(theta) + y * math.sin(theta))
    yr = (-x * math.sin(theta) + y * math.cos(theta))

    return xr, yr

def inside_ellipse(lat, lon):
    x, y = gps_to_local(lat, lon, CENTER_LAT, CENTER_LON)
    xr, yr = rotate_point(x, y, ROTATION)
    value = ((xr ** 2) / (MAJOR_AXIS ** 2) + (yr ** 2) / (MINOR_AXIS ** 2))

    return value <= 1.0, value

# ================================================
# SERVO TRIGGER
# ================================================
def outside_geofence_trigger():
    global servo_triggered

    # Safety check
    if servo_triggered:
        print("Servo has already been triggered. Ignoring additional trigger request.")
        logging.warning(
            "Additional servo trigger ignored."
        )

        return

    servo_triggered = True

    print()
    print("========================================")
    print(">>> GEOFENCE TRIGGER ACTIVATED <<<")
    print("========================================")
    logging.warning(
        "Geofence trigger activated."
    )

    try:
        print(f"Moving servo to ACTIVATED position ({ACTIVATE_POSITION})...")
        logging.warning(
            f"Servo moving to activation position: "
            f"{ACTIVATE_POSITION}"
        )
        servo.value = ACTIVATE_POSITION

        print(f"Servo is ACTIVATED. Holding for {ACTIVATE_TIME} seconds.")
        logging.warning(
            f"Servo activated. "
            f"Holding for {ACTIVATE_TIME} seconds."
        )
        time.sleep(ACTIVATE_TIME)

        if RETURN_TO_NORMAL:
            print("Activation duration complete.")

            print(f"Returning servo to IDLE position ({NORMAL_POSITION})...")
            logging.info(
                "Activation duration complete. "
                "Returning servo to idle."
            )
            servo.value = NORMAL_POSITION
            time.sleep(MOVE_TIME)

            print("Servo returned to IDLE position.")
            logging.info(
                "Servo returned to idle position."
            )

        else:
            print("Servo will remain in ACTIVATED position.")
            logging.info(
                "Servo remains in activated position."
            )

    except Exception as error:
        print(f"ERROR: Servo activation failed: {error}")
        logging.exception(
            f"Servo activation error: {error}"
        )
        raise


# ================================================
# GEOFENCE STATUS
# ================================================
def check_geofence(lat, lon):
    # -----Circular-----
    if GEOFENCE_TYPE == 1:
        distance = haversine(CENTER_LAT, CENTER_LON, lat, lon)
        inside = (distance <= RADIUS_METERS)

        print(f"Distance from centre: {distance:.2f} metres")
        print(f"Geofence radius: {RADIUS_METERS:.2f} metres")
        logging.info(
            f"Circular geofence | "
            f"Distance={distance:.2f}m | "
            f"Radius={RADIUS_METERS}m"
        )

        return inside

    # -----Polygon-----
    elif GEOFENCE_TYPE == 2:
        inside = point_inside_polygon(lat, lon, POLYGON)
        return inside

    # Ellipse
    else:
        inside, value = inside_ellipse(lat, lon)
        print(f"Ellipse value: {value:.4f} (inside if <= 1.0000)")
        logging.info(
            f"Elliptical geofence | "
            f"Ellipse value={value:.4f}"
        )

        return inside


# ================================================
# MAIN LOOP
# ================================================
def main():
    global outside_count
    print("\n========================================")
    print("       GEOFENCE MONITORING SYSTEM")
    print("========================================\n")
    logging.info(
        "Geofence monitoring started."
    )

    # -----Display geofence type-----
    if GEOFENCE_TYPE == 1:
        print("Geofence type: CIRCULAR")
        print(f"Centre: ({CENTER_LAT}, {CENTER_LON})")
        print(f"Radius: {RADIUS_METERS} metres")
        logging.info(
            "Using circular geofence."
        )

    elif GEOFENCE_TYPE == 2:
        print("Geofence type: POLYGON")
        print(f"Polygon points: {len(POLYGON)}")
        logging.info(
            "Using polygon geofence."
        )

    else:
        print("Geofence type: ELLIPTICAL")
        print(f"Centre: ({CENTER_LAT}, {CENTER_LON})")
        print(f"Major axis: {MAJOR_AXIS} metres")
        print(f"Minor axis: {MINOR_AXIS} metres")
        print(f"Rotation: {ROTATION} degrees")
        logging.info(
            "Using elliptical geofence."
        )

    # -----Display GPS configuration-----
    print("\nGPS configuration:")
    print(f"\tGPS file: {LATEST_GPS_FILE}")
    print(f"\tSmoothing samples: {GPS_SMOOTHING_SAMPLES}")
    print(f"\tOutside confirmations required: {OUTSIDE_CONFIRMATIONS_REQUIRED}")
    print(f"\tCheck interval: {CHECK_INTERVAL} seconds")

    # -----Servo configuration-----
    print("\nServo configuration:")
    print(f"\tGPIO pin: {SERVO_PIN}")
    print(f"\tIdle position: {NORMAL_POSITION}")
    print(f"\tActivated position: {ACTIVATE_POSITION}")
    print(f"\tActivation duration: {ACTIVATE_TIME} seconds")

    print("\n----------------------------------------")
    print("Starting geofence monitoring...")
    print("----------------------------------------\n")

    # -----CONTINUOUS MONITORING-----
    while True:
        # Get GPS location
        lat, lon = get_current_location()

        print("\nCurrent smoothed coordinates:")
        print(f"\tLatitude : {lat:.6f}\n\tLongitude: {lon:.6f}")

        # Check geofence
        inside = check_geofence(lat, lon)

        if inside:
            outside_count = 0

            print("\nSTATUS: INSIDE GEOFENCE")
            print(f"Coordinates: ({lat:.6f}, {lon:.6f})")
            logging.info(
                f"INSIDE | "
                f"Position=({lat:.6f}, {lon:.6f})"
            )

        else:
            outside_count += 1

            print("\nSTATUS: OUTSIDE GEOFENCE")
            print(f"Coordinates: ({lat:.6f}, {lon:.6f})")
            print(f"Outside confirmation: {outside_count}/{OUTSIDE_CONFIRMATIONS_REQUIRED}")
            logging.warning(
                f"OUTSIDE | "
                f"Position=({lat:.6f}, {lon:.6f}) | "
                f"Confirmation "
                f"{outside_count}/"
                f"{OUTSIDE_CONFIRMATIONS_REQUIRED}"
            )

            if (outside_count >= OUTSIDE_CONFIRMATIONS_REQUIRED):
                print("\n========================================")
                print("OUTSIDE GEOFENCE CONFIRMED")
                print(f"Final coordinates: ({lat:.6f}, {lon:.6f})")
                print("Activating servo...")
                print("========================================")

                outside_geofence_trigger()
                logging.info(
                    "Program terminating after confirmed geofence exit."
                )
                break

            else:
                print("Outside reading not yet confirmed.")
                print("Waiting for another stable GPS reading.")

        # Wait until next check
        print(f"\nNext GPS/geofence check in {CHECK_INTERVAL} seconds...")
        time.sleep(CHECK_INTERVAL)


# ================================================
# PROGRAM CLEANUP
# ================================================
def cleanup():
    global servo
    try:
        if servo is not None:
            print("\nCleaning up servo...")
            servo.value = NORMAL_POSITION
            time.sleep(MOVE_TIME)
            servo.close()

            print("Servo cleanup complete.")
            logging.info("Servo cleanup complete.")

    except Exception as error:
        print(f"WARNING: Servo cleanup error: {error}")
        logging.exception(
            f"Servo cleanup error: {error}"
        )


# ================================================
# PROGRAM ENTRY POINT
# ================================================
if __name__ == "__main__":
    try:
        # Initialise servo before starting GPS/geofence monitoring.
        initialise_servo()
        main()

    except KeyboardInterrupt:
        print("\n========================================")
        print("\nGeofence monitoring stopped by user.")
        print("\n========================================")
        logging.info(
            "Geofence monitoring stopped by user."
        )

    except Exception as error:
        print("\n========================================")
        print(f"FATAL ERROR: {error}")
        print("\n========================================")
        logging.exception(
            f"Unexpected geofence monitoring error: {error}"
        )

    finally:
        cleanup()
        print("\nGeofence program terminated.")
        logging.info(
            "Geofence program terminated."
        )