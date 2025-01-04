import numpy as np
from copy import copy

# TODO: Dobra, jak jeszcze mogę rozróżniać przeszkody? - Do czego innego może służyć RHT?


def plane_from_points(p1, p2, p3):
    """
    Compute the plane parameters (a, b, c, d) from three points in 3D space.

    Args:
        p1, p2, p3: Lists or arrays representing three points in 3D space [x, y, z].

    Returns:
        A tuple (a, b, c, d) representing the plane equation: ax + by + cz + d = 0.
    """
    p1, p2, p3 = np.array(p1), np.array(p2), np.array(p3)

    # two vectors lying on the plane
    v1 = p2 - p1
    v2 = p3 - p1

    # normal vector to the plane
    c_product = np.cross(v1, v2)
    # if not c_product.any():
    #     return None
    magnitude = np.linalg.norm(c_product)  # Computes the magnitude directly
    if magnitude < 1e-8:
        print("The points are collinear or very close to collinear; cannot compute a valid plane.")
        return None

    c_product /= magnitude

    a, b, c = c_product
    # d by substituting point p1 into the plane equation
    d = -np.dot(c_product, p1)

    return a, b, c, d


def rht_planes2(points, num_samples, bins, min_plane_points, min_distance):
    res = 1 / bins

    # Normalization of ts column to <new_min; new_max>
    new_min = 0
    new_max = 100
    old_min = np.max(points[:, 0])
    old_max = np.min(points[:, 0])
    col = points[:, 0]
    points[:, 0] = ((col - col.min()) / (old_max - old_min)) * (new_max - new_min) + new_min
    
    hough_accumulator = dict()  # Hough Space of plane parameters {(a, b, c, d) : <n_votes>, ...}
    planes = dict()  # {(a, b, c, d) : np.ndarray([[x1,y1], ...]), ...}  # planes to points of planes

    for i in range(num_samples):
        # Randomly sample 3 points
        sample_points = np.zeros((3,3))
        sample_frame = 40
        sample_points[0] = points[np.random.choice(points.shape[0], 1, replace=False)]
        to_sample_points = points[(sample_points[0,1] - sample_frame / 2) < points[:,1]]
        to_sample_points = to_sample_points[to_sample_points[:,1] < (sample_points[0,1] + sample_frame / 2)]
        to_sample_points = to_sample_points[(sample_points[0,2] - sample_frame / 2) < to_sample_points[:,2]]
        to_sample_points = to_sample_points[to_sample_points[:,2] < (sample_points[0,2] + sample_frame / 2)]
        if to_sample_points.shape[0] <= 1:
            continue

        rest_points = to_sample_points[np.random.choice(to_sample_points.shape[0], 2, replace=False)]
        sample_points[1] = rest_points[0]
        sample_points[2] = rest_points[1]

        # Calculate the plane parameters
        plane = plane_from_points(sample_points[0], sample_points[1], sample_points[2])
        if plane is None:
            continue
        
        a, b, c, d = plane

        # Discretization with dynamic rsesolution (quantization)

        # a_disc = saturation(dynamic_qauantization(a, res), saturation_val)
        # b_disc = saturation(dynamic_qauantization(b, res), saturation_val)
        # c_disc = saturation(dynamic_qauantization(c, res), saturation_val)
        # d_disc = saturation(dynamic_qauantization(d, res), saturation_val)

        a_disc = dynamic_qauantization(a, res)
        b_disc = dynamic_qauantization(b, res)
        c_disc = dynamic_qauantization(c, res)
        d_disc = dynamic_qauantization(d, res)

        # a_disc = quantize(a, res)
        # b_disc = quantize(b, res)
        # c_disc = quantize(c, res)
        # d_disc = quantize(d, res)

        discretized_plane = (a_disc, b_disc, c_disc, d_disc)

        print(plane)
        print(discretized_plane)
        
        # Vote in Hough space
        if discretized_plane in hough_accumulator:
            continue
        else:
            a, b, c, d = discretized_plane
            # Points distances from plane
            # Numerator: |ax + by + cz + d|
            num = np.abs(a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d)
            # Denominator: sqrt(a^2 + b^2 + c^2)
            den = np.sqrt(a**2 + b**2 + c**2)
            distances = num / den
            
            indexes = distances <= min_distance
            plane_points = points[indexes]
            hough_accumulator[discretized_plane] = plane_points.shape[0]

            if plane_points.shape[0] >= min_plane_points:
                planes[(a,b,c,d)] = plane_points
                # points = points[np.logical_not(indexes)]

        if points.shape[0] < 3:
            break
            
    # Denormalization if needed
    # col = points[:, 0]
    # points[:, 0] = ((col - new_min) / (new_max - new_min)) * (old_max - old_min) + old_min

    return planes



def rht_planes(points: np.ndarray, num_samples, bins, min_votes, min_plane_points, num_candidates, min_distance, saturation_val=100):
    """
    Perform Random Hough Transform (RHT) to fit planes to a 3D point cloud
    
    Arguments:
    - points: A numpy array of shape (N, 3) representing the 3D point cloud
    - num_samples: number of iterations
    - bins: Number of bins to discretize the Hough space for voting; eg. 5 means discretization with resolution 0.2 for >1 <10 values, 2 for >10 <100 values, etc.
    - min_votes: min number of votes to recognize plane as a solution candidate
    - min_plane_points: min value of points fitting plane equation to recognize as one of solutions
    - num_candidates: number of solution candidates to find before searching for solution  
    - min_distance: min distance from point to plane to recognize it as on-plane point
    - max_parameters_val: value of parameters saturation
    
    Returns:
    - A list of detected plane parameters (a, b, c, d) corresponding to peaks in Hough space.
    """
    # Czy trzeba kopiować points?

    res = 1 / bins

    # Normalization of ts column to <new_min; new_max>
    new_min = 0
    new_max = 1
    old_min = np.max(points[:, 0])
    old_max = np.min(points[:, 0])
    col = points[:, 0]
    points[:, 0] = ((col - col.min()) / (old_max - old_min)) * (new_max - new_min) + new_min
    
    hough_accumulator = dict()  # Hough Space of plane parameters {(a, b, c, d) : <n_votes>, ...}
    candidates = list()
    planes = dict()  # {(a, b, c, d) : np.ndarray([[x1,y1], ...]), ...}  # planes to points of planes
    discarded_candidates = set()
    for i in range(num_samples):
        # Randomly sample 3 points
        sample_points = np.zeros((3,3))
        sample_frame = 40
        sample_points[0] = points[np.random.choice(points.shape[0], 1, replace=False)]
        to_sample_points = points[(sample_points[0,1] - sample_frame / 2) < points[:,1]]
        to_sample_points = to_sample_points[to_sample_points[:,1] < (sample_points[0,1] + sample_frame / 2)]
        to_sample_points = to_sample_points[(sample_points[0,2] - sample_frame / 2) < to_sample_points[:,2]]
        to_sample_points = to_sample_points[to_sample_points[:,2] < (sample_points[0,2] + sample_frame / 2)]
        if to_sample_points.shape[0] <= 1:
            continue

        rest_points = to_sample_points[np.random.choice(to_sample_points.shape[0], 2, replace=False)]
        sample_points[1] = rest_points[0]
        sample_points[2] = rest_points[1]

        # Calculate the plane parameters
        plane = plane_from_points(sample_points[0], sample_points[1], sample_points[2])
        if plane is None:
            continue
        
        a, b, c, d = plane

        # Discretization with dynamic rsesolution (quantization)
        # TODO: Prztestować w gazebo + znaleźć zbiór danych z samochodami, może trzeba będzie usunąć saturację
        # Potem dobrać parametry RHT i uzależnić od liczby eventów (opc)
        # Potem trzeba będzie ogarną wizualizację - mają być widoczne wszystkie obiekty
        # Potem przeszkody jako obiekty - przypisywać im indeks, kolor do wizualizacji, ich eventy
        # Potem kolorowanie eventów należących do jednej przeszkody - event feame jako RGB - trzeba będzie jakoś rozpoznawać, że to ta sama - operator "=="?
        # Zrobić żeby v2e aproksymało do 1000 fps, a kamera dokładnie tyle ile mają ich filmiki i przetestować - niech dron lata prawo - lewo
        # Potem detekcja rogów

        # a_disc = saturation(dynamic_qauantization(a, res), saturation_val)
        # b_disc = saturation(dynamic_qauantization(b, res), saturation_val)
        # c_disc = saturation(dynamic_qauantization(c, res), saturation_val)
        # d_disc = saturation(dynamic_qauantization(d, res), saturation_val)

        a_disc = dynamic_qauantization(a, res)
        b_disc = dynamic_qauantization(b, res)
        c_disc = dynamic_qauantization(c, res)
        d_disc = dynamic_qauantization(d, res)

        # a_disc = quantize(a, res)
        # b_disc = quantize(b, res)
        # c_disc = quantize(c, res)
        # d_disc = quantize(d, res)

        discretized_plane = (a_disc, b_disc, c_disc, d_disc)

        print(plane)
        print(discretized_plane)
        
        # Vote in Hough space
        if discretized_plane in hough_accumulator:
            hough_accumulator[discretized_plane] += 1
        else:
            hough_accumulator[discretized_plane] = 1

        # Recognizing as a solution candidate
        if hough_accumulator[discretized_plane] >= min_votes and discretized_plane not in candidates and discretized_plane not in planes and discretized_plane not in discarded_candidates:
            candidates.append(discretized_plane)

        # Searching for solutions
        if len(candidates) >= num_candidates or i == num_samples - 1:
            candidates.sort(key=lambda plane: hough_accumulator[plane], reverse=True)
            for (a, b, c, d) in candidates:
                # Points distances from plane
                # Numerator: |ax + by + cz + d|
                num = np.abs(a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d)
                # Denominator: sqrt(a^2 + b^2 + c^2)
                den = np.sqrt(a**2 + b**2 + c**2)
                distances = num / den
                
                indexes = distances <= min_distance
                candidate_points = points[indexes]

                print("votes:", hough_accumulator[(a,b,c,d)])
                print("candidate_points n:", candidate_points.shape[0])

                # Solution found
                if candidate_points.shape[0] >= min_plane_points:
                    planes[(a,b,c,d)] = candidate_points
                    points = points[np.logical_not(indexes)]
                else:  # Discard candidate
                    discarded_candidates.add((a,b,c,d))

            for d_plane in discarded_candidates:
                if d_plane in candidates:
                    candidates.remove(d_plane)

        if points.shape[0] < 3:
            break
            
    # Denormalization if needed
    # col = points[:, 0]
    # points[:, 0] = ((col - new_min) / (new_max - new_min)) * (old_max - old_min) + old_min

    return planes


def quantize(x: float, res: float) -> float:
    return np.round(x / res) * res

def dynamic_qauantization(x: float, one_int_digit_res: float) -> float:
    x_abs = np.abs(x)

    if x_abs >= 1:
        x_row = len(np.format_float_positional(x_abs).split('.')[0].lstrip('-')) - 1
    else:
        x_str = np.format_float_positional(x_abs).split('.')[-1]
        x_row = -(len(x_str) - len(x_str.lstrip('0')) + 1)

    x_res = one_int_digit_res * 10**x_row
    x_out = np.round(x / (x_res)) * (x_res)

    # print(x)
    # print(x_out)

    return x_out


def saturation(x, sat):
    '''
    Limit x to range <-sat; -1/sat> + <1/sat; sat>
    '''
    x_abs = np.abs(x)
    x_sat = x_abs
    if x_abs < 1/sat:
        x_sat = 0
    elif x_abs > sat:
        x_sat = sat

    return x_sat * np.sign(x)
