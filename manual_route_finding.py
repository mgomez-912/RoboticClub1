import random
import itertools
import time

# --- Grid Configuration ---
GRID_ROWS = 4
GRID_COLS = 5
START_ROBOT_POS = (0, 2)  # (row, col) - 0-indexed, Row 1, Col 3 from your image

# Define the 4 specific Red Zone coordinates for delivery
# Assuming they are in a virtual row 4, at columns 0, 1, 3, 4 based on image gaps.
DELIVERY_ROW = 4  # Extend the grid conceptually for delivery
RED_ZONE_COORDS = [(DELIVERY_ROW, 0), (DELIVERY_ROW, 1), (DELIVERY_ROW, 3), (DELIVERY_ROW, 4)]


# --- Helper Functions ---

def manhattan_distance(pos1, pos2):
    """Calculates Manhattan distance between two points."""
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1]);

get_distance = manhattan_distance;

def get_user_coordinates(num_objects, object_type, existing_coords):
    """
    Prompts the user to enter coordinates for objects, with validation.
    Ensures coordinates are within bounds and unique.
    """
    objects = []
    print(f"\nEnter coordinates for {num_objects} {object_type} objects (Row and Column):");
    print(f"  Valid rows: 1, 2, 3 (0-indexed: 1-3)");
    print(f"  Valid columns: 0, 1, 2, 3, 4");

    for i in range(num_objects):
        while True:
            try:
                row_str = input(f"  {object_type} object {i+1} - Enter row: ");
                col_str = input(f"  {object_type} object {i+1} - Enter column: ");

                row = int(row_str);
                col = int(col_str);

                # Validate row: 1, 2, or 3 (0-indexed)
                if not (1 <= row < GRID_ROWS): # GRID_ROWS is 4, so rows 1, 2, 3 are valid
                    print(f"Error: Row {row} is out of valid range (1-3). Please try again.");
                    continue;

                # Validate column: 0-4
                if not (0 <= col < GRID_COLS):
                    print(f"Error: Column {col} is out of valid range (0-4). Please try again.");
                    continue;

                pos = (row, col);

                # Check for uniqueness
                if pos in objects or pos in existing_coords:
                    print(f"Error: Position {pos} is already taken. Please enter unique coordinates.");
                    continue;

                objects.append(pos);
                existing_coords.add(pos); # Add to set of all used coordinates
                break; # Exit inner while loop if coordinates are valid and unique

            except ValueError:
                print("Invalid input. Please enter integer values for row and column.");
            except Exception as e:
                print(f"An unexpected error occurred: {e}");
    return objects;


def calculate_trip_cost(start_pos, pairs_in_trip_list, assigned_red_zones_for_trip):
    """
    Calculates the minimum cost for a single trip (starting at start_pos, picking up
    1 or 2 pairs, and delivering to the specified assigned red zones).

    Args:
        start_pos (tuple): The robot's starting (row, col) for this trip.
        pairs_in_trip_list (list): A list of (blue_pos, red_pos) tuples for pairs in this trip.
                                    Expected to be 1 or 2 pairs.
        assigned_red_zones_for_trip (list): A list of (row, col) for the specific red zones
                                             this trip must deliver to. Expected to be 1 or 2 zones.

    Returns:
        tuple: (total_trip_cost, final_delivery_pos, trip_sequence_description_list)
               Returns float('inf') if an invalid number of pairs/zones is given.
    """
    
    current_trip_min_pickup_cost = float('inf');
    best_pickup_sequence_in_trip = [];
    pickup_end_pos = None;  # Position after collecting all objects in the trip

    # --- Phase 1: Object Pickups ---
    if len(pairs_in_trip_list) == 1:
        if len(assigned_red_zones_for_trip) != 1:
            return float('inf'), None, []; # Mismatch: 1 pair needs 1 red zone
        
        b_pos, r_pos = pairs_in_trip_list[0];
        
        # Calculate cost for picking up one blue then one red object
        current_trip_min_pickup_cost = get_distance(start_pos, b_pos) + get_distance(b_pos, r_pos);
        pickup_end_pos = r_pos;
        best_pickup_sequence_in_trip = [f"Pick Blue {b_pos}", f"Pick Red {r_pos}"];

    elif len(pairs_in_trip_list) == 2:
        if len(assigned_red_zones_for_trip) != 2:
            return float('inf'), None, []; # Mismatch: 2 pairs need 2 red zones

        (b1_pos, r1_pos) = pairs_in_trip_list[0];
        (b2_pos, r2_pos) = pairs_in_trip_list[1];
        
        all_objects_in_trip = [b1_pos, r1_pos, b2_pos, r2_pos];
        
        # Find the best order to visit these 4 objects (mini-TSP for pickup)
        for object_pickup_order_tuple in itertools.permutations(all_objects_in_trip):
            current_sequence_cost = 0;
            current_pickup_pos = start_pos;
            current_pickup_path = [];
            
            for obj_pos in object_pickup_order_tuple:
                current_sequence_cost += get_distance(current_pickup_pos, obj_pos);
                current_pickup_pos = obj_pos;
                current_pickup_path.append(f"Pick {obj_pos}");
            
            if current_sequence_cost < current_trip_min_pickup_cost:
                current_trip_min_pickup_cost = current_sequence_cost;
                best_pickup_sequence_in_trip = current_pickup_path;
                pickup_end_pos = current_pickup_pos; # The last object picked up

    else:
        # Invalid number of pairs for a single trip (only 1 or 2 allowed)
        return float('inf'), None, [];

    # --- Phase 2: Delivery to Red Zones ---
    delivery_cost = float('inf');
    best_delivery_sequence = [];
    delivery_end_pos = None;

    if len(assigned_red_zones_for_trip) == 1:
        red_zone_pos = assigned_red_zones_for_trip[0];
        delivery_cost = get_distance(pickup_end_pos, red_zone_pos);
        best_delivery_sequence = [f"Deliver to Red Zone {red_zone_pos}"];
        delivery_end_pos = red_zone_pos;

    elif len(assigned_red_zones_for_trip) == 2:
        rz1_pos, rz2_pos = assigned_red_zones_for_trip[0], assigned_red_zones_for_trip[1];
        
        # Option 1: Pickup end -> RZ1 -> RZ2 (Deliver to RZ1 first, then RZ2)
        cost_option1 = get_distance(pickup_end_pos, rz1_pos) + get_distance(rz1_pos, rz2_pos);
        path_option1 = [f"Deliver to Red Zone {rz1_pos}", f"Then deliver to Red Zone {rz2_pos}"];
        
        # Option 2: Pickup end -> RZ2 -> RZ1 (Deliver to RZ2 first, then RZ1)
        cost_option2 = get_distance(pickup_end_pos, rz2_pos) + get_distance(rz2_pos, rz1_pos);
        path_option2 = [f"Deliver to Red Zone {rz2_pos}", f"Then deliver to Red Zone {rz1_pos}"];

        if cost_option1 < cost_option2:
            delivery_cost = cost_option1;
            best_delivery_sequence = path_option1;
            delivery_end_pos = rz2_pos; # Robot ends up at the last visited red zone
        else:
            delivery_cost = cost_option2;
            best_delivery_sequence = path_option2;
            delivery_end_pos = rz1_pos; # Robot ends up at the last visited red zone
    
    else:
        # Invalid number of red zones for a trip (only 1 or 2 allowed)
        return float('inf'), None, [];


    total_trip_cost = current_trip_min_pickup_cost + delivery_cost;
    
    trip_sequence_desc = [f"Robot starts at {start_pos}"] + best_pickup_sequence_in_trip + best_delivery_sequence;
    
    return total_trip_cost, delivery_end_pos, trip_sequence_desc;


def solve_robot_challenge_with_red_zones():
    
    # Use a set to keep track of all occupied positions (blue and red) to ensure uniqueness
    all_occupied_positions = set();

    blue_objects = get_user_coordinates(4, "Blue", all_occupied_positions);
    red_objects = get_user_coordinates(4, "Red", all_occupied_positions);

    print(f"\nUser-provided Blue Objects: {blue_objects}");
    print(f"User-provided Red Objects: {red_objects}");
    print(f"Red Zone Delivery Points: {RED_ZONE_COORDS}");

    # --- Start timing the route calculation ---
    start_time = time.time();

    min_total_distance = float('inf');
    best_overall_path_sequence = [];

    # Indices for blue and red objects to manage pairings
    blue_indices = list(range(len(blue_objects)));
    red_indices = list(range(len(red_objects)));

    # Indices for the red zone coordinates
    red_zone_indices = list(range(len(RED_ZONE_COORDS))); # Expected to be [0, 1, 2, 3]

    # Outer loop: Iterate through all possible fixed pairings of blue and red objects (4! ways)
    for red_pairing_permutation in itertools.permutations(red_indices):
        current_fixed_pairs = []; # List of (blue_pos, red_pos) tuples for this specific pairing
        for i in range(len(blue_objects)):
            current_fixed_pairs.append((blue_objects[i], red_objects[red_pairing_permutation[i]]));

        # Now `current_fixed_pairs` holds 4 specific (B,R) pairs. E.g., [P0, P1, P2, P3]

        # Next layer: Iterate through all possible assignments of these 4 pairs to the 4 red zones (4! ways)
        # This means, for this `red_zone_assignment_permutation`, current_fixed_pairs[0] will be assigned to
        # RED_ZONE_COORDS[red_zone_assignment_permutation[0]], current_fixed_pairs[1] to
        # RED_ZONE_COORDS[red_zone_assignment_permutation[1]], and so on.
        for red_zone_assignment_permutation in itertools.permutations(red_zone_indices):
            
            # Create a list of tuples: (blue_pos, red_pos, assigned_red_zone_pos)
            pairs_with_assigned_targets = [];
            for i in range(len(current_fixed_pairs)):
                pairs_with_assigned_targets.append((current_fixed_pairs[i][0], current_fixed_pairs[i][1],
                                                     RED_ZONE_COORDS[red_zone_assignment_permutation[i]]));

            # Inner loop: Iterate through all permutations of the order in which these 4
            # (pair, assigned_target_red_zone) tuples are delivered. This is crucial for batching.
            
            # These are the indices into `pairs_with_assigned_targets`
            delivery_order_indices = list(range(len(pairs_with_assigned_targets)));

            for ordered_delivery_of_pairs in itertools.permutations(delivery_order_indices):
                # `ordered_delivery_of_pairs` now specifies the sequence of pairs to deliver.
                # E.g., (0, 1, 2, 3) means deliver pair_with_target[0], then [1], then [2], then [3].
                
                # --- For this specific `ordered_delivery_of_pairs` sequence, try different batching strategies ---

                # Strategy 1: Four 1-pair trips
                current_cost_strategy1 = 0;
                current_robot_loc_strategy1 = START_ROBOT_POS;
                current_path_strategy1 = [];
                
                is_valid_strategy1 = True; # Track if this strategy is feasible (e.g., if calculate_trip_cost returns inf)
                for i in range(4): # Process each pair individually
                    # Get the data for the current pair to be delivered
                    pair_idx_in_original_list = ordered_delivery_of_pairs[i];
                    blue_pos, red_pos, target_rz = pairs_with_assigned_targets[pair_idx_in_original_list];
                    
                    cost_trip, end_pos_after_delivery, trip_desc = calculate_trip_cost(current_robot_loc_strategy1,
                                                                                       [(blue_pos, red_pos)],
                                                                                       [target_rz]);
                    
                    if cost_trip == float('inf'): # If any trip is invalid, this whole strategy is invalid
                        is_valid_strategy1 = False;
                        break;
                    
                    current_cost_strategy1 += cost_trip;
                    current_robot_loc_strategy1 = end_pos_after_delivery; # Robot ends up at the delivered red zone
                    current_path_strategy1.extend(trip_desc);
                
                if is_valid_strategy1 and current_cost_strategy1 < min_total_distance:
                    min_total_distance = current_cost_strategy1;
                    best_overall_path_sequence = current_path_strategy1;
                    # print(f"New best (4x1 delivery): {min_total_distance}")


                # Strategy 2: Two 2-pair trips
                # This applies only if we have exactly 4 pairs.
                if len(ordered_delivery_of_pairs) == 4:
                    current_cost_strategy2 = 0;
                    current_robot_loc_strategy2 = START_ROBOT_POS;
                    current_path_strategy2 = [];
                    
                    is_valid_strategy2 = True;

                    # First 2-pair trip (using the first two pairs in the ordered_delivery_of_pairs sequence)
                    pair1_idx = ordered_delivery_of_pairs[0];
                    pair2_idx = ordered_delivery_of_pairs[1];
                    
                    pair1_data = pairs_with_assigned_targets[pair1_idx]; # (b1, r1, target_rz1)
                    pair2_data = pairs_with_assigned_targets[pair2_idx]; # (b2, r2, target_rz2)
                    
                    cost_trip1, end_pos_trip1, trip_desc1 = calculate_trip_cost(current_robot_loc_strategy2,
                                                                                [(pair1_data[0], pair1_data[1]), (pair2_data[0], pair2_data[1])],
                                                                                [pair1_data[2], pair2_data[2]]);
                    
                    if cost_trip1 == float('inf'):
                        is_valid_strategy2 = False;
                    else:
                        current_cost_strategy2 += cost_trip1;
                        current_robot_loc_strategy2 = end_pos_trip1;
                        current_path_strategy2.extend(trip_desc1);

                    # Second 2-pair trip (using the last two pairs in the ordered_delivery_of_pairs sequence)
                    if is_valid_strategy2:
                        pair3_idx = ordered_delivery_of_pairs[2];
                        pair4_idx = ordered_delivery_of_pairs[3];

                        pair3_data = pairs_with_assigned_targets[pair3_idx];
                        pair4_data = pairs_with_assigned_targets[pair4_idx];

                        cost_trip2, end_pos_trip2, trip_desc2 = calculate_trip_cost(current_robot_loc_strategy2,
                                                                                    [(pair3_data[0], pair3_data[1]), (pair4_data[0], pair4_data[1])],
                                                                                    [pair3_data[2], pair4_data[2]]);
                        if cost_trip2 == float('inf'):
                            is_valid_strategy2 = False;
                        else:
                            current_cost_strategy2 += cost_trip2;
                            current_robot_loc_strategy2 = end_pos_trip2;
                            current_path_strategy2.extend(trip_desc2);

                    if is_valid_strategy2 and current_cost_strategy2 < min_total_distance:
                        min_total_distance = current_cost_strategy2;
                        best_overall_path_sequence = current_path_strategy2;
                        # print(f"New best (2x2 delivery): {min_total_distance}")


                # Strategy 3: One 2-pair trip and two 1-pair trips (3 permutations of this)

                # Option 3a: First two pairs as a 2-pair trip, then two 1-pair trips
                if len(ordered_delivery_of_pairs) == 4:
                    current_cost_3a = 0;
                    current_robot_loc_3a = START_ROBOT_POS;
                    current_path_3a = [];
                    is_valid_3a = True;

                    # Trip 1: 2-pair
                    pair1_idx = ordered_delivery_of_pairs[0];
                    pair2_idx = ordered_delivery_of_pairs[1];
                    pair1_data = pairs_with_assigned_targets[pair1_idx];
                    pair2_data = pairs_with_assigned_targets[pair2_idx];
                    cost_trip1, end_pos_trip1, trip_desc1 = calculate_trip_cost(current_robot_loc_3a,
                                                                                [(pair1_data[0], pair1_data[1]), (pair2_data[0], pair2_data[1])],
                                                                                [pair1_data[2], pair2_data[2]]);
                    if cost_trip1 == float('inf'):
                        is_valid_3a = False;
                    else:
                        current_cost_3a += cost_trip1;
                        current_robot_loc_3a = end_pos_trip1;
                        current_path_3a.extend(trip_desc1);

                    # Trip 2: 1-pair
                    if is_valid_3a:
                        pair3_idx = ordered_delivery_of_pairs[2];
                        pair3_data = pairs_with_assigned_targets[pair3_idx];
                        cost_trip2, end_pos_trip2, trip_desc2 = calculate_trip_cost(current_robot_loc_3a,
                                                                                    [(pair3_data[0], pair3_data[1])],
                                                                                    [pair3_data[2]]);
                        if cost_trip2 == float('inf'):
                            is_valid_3a = False;
                        else:
                            current_cost_3a += cost_trip2;
                            current_robot_loc_3a = end_pos_trip2;
                            current_path_3a.extend(trip_desc2);

                    # Trip 3: 1-pair
                    if is_valid_3a:
                        pair4_idx = ordered_delivery_of_pairs[3];
                        pair4_data = pairs_with_assigned_targets[pair4_idx];
                        cost_trip3, end_pos_trip3, trip_desc3 = calculate_trip_cost(current_robot_loc_3a,
                                                                                    [(pair4_data[0], pair4_data[1])],
                                                                                    [pair4_data[2]]);
                        if cost_trip3 == float('inf'):
                            is_valid_3a = False;
                        else:
                            current_cost_3a += cost_trip3;
                            current_robot_loc_3a = end_pos_trip3;
                            current_path_3a.extend(trip_desc3);

                    if is_valid_3a and current_cost_3a < min_total_distance:
                        min_total_distance = current_cost_3a;
                        best_overall_path_sequence = current_path_3a;
                        # print(f"New best (1x2 + 2x1 delivery): {min_total_distance}")

                # Option 3b: Middle two pairs as a 2-pair trip
                if len(ordered_delivery_of_pairs) == 4:
                    current_cost_3b = 0;
                    current_robot_loc_3b = START_ROBOT_POS;
                    current_path_3b = [];
                    is_valid_3b = True;

                    # Trip 1: 1-pair (first pair)
                    pair1_idx = ordered_delivery_of_pairs[0];
                    pair1_data = pairs_with_assigned_targets[pair1_idx];
                    cost_trip1, end_pos_trip1, trip_desc1 = calculate_trip_cost(current_robot_loc_3b,
                                                                                [(pair1_data[0], pair1_data[1])],
                                                                                [pair1_data[2]]);
                    if cost_trip1 == float('inf'):
                        is_valid_3b = False;
                    else:
                        current_cost_3b += cost_trip1;
                        current_robot_loc_3b = end_pos_trip1;
                        current_path_3b.extend(trip_desc1);

                    # Trip 2: 2-pair (middle two pairs)
                    if is_valid_3b:
                        pair2_idx = ordered_delivery_of_pairs[1];
                        pair3_idx = ordered_delivery_of_pairs[2];
                        pair2_data = pairs_with_assigned_targets[pair2_idx];
                        pair3_data = pairs_with_assigned_targets[pair3_idx];
                        cost_trip2, end_pos_trip2, trip_desc2 = calculate_trip_cost(current_robot_loc_3b,
                                                                                    [(pair2_data[0], pair2_data[1]), (pair3_data[0], pair3_data[1])],
                                                                                    [pair2_data[2], pair3_data[2]]);
                        if cost_trip2 == float('inf'):
                            is_valid_3b = False;
                        else:
                            current_cost_3b += cost_trip2;
                            current_robot_loc_3b = end_pos_trip2;
                            current_path_3b.extend(trip_desc2);

                    # Trip 3: 1-pair (last pair)
                    if is_valid_3b:
                        pair4_idx = ordered_delivery_of_pairs[3];
                        pair4_data = pairs_with_assigned_targets[pair4_idx];
                        cost_trip3, end_pos_trip3, trip_desc3 = calculate_trip_cost(current_robot_loc_3b,
                                                                                    [(pair4_data[0], pair4_data[1])],
                                                                                    [pair4_data[2]]);
                        if cost_trip3 == float('inf'):
                            is_valid_3b = False;
                        else:
                            current_cost_3b += cost_trip3;
                            current_robot_loc_3b = end_pos_trip3;
                            current_path_3b.extend(trip_desc3);

                    if is_valid_3b and current_cost_3b < min_total_distance:
                        min_total_distance = current_cost_3b;
                        best_overall_path_sequence = current_path_3b;
                        # print(f"New best (1x1 + 1x2 + 1x1 delivery): {min_total_distance}")

                # Option 3c: Last two pairs as a 2-pair trip
                if len(ordered_delivery_of_pairs) == 4:
                    current_cost_3c = 0;
                    current_robot_loc_3c = START_ROBOT_POS;
                    current_path_3c = [];
                    is_valid_3c = True;

                    # Trip 1: 1-pair (first pair)
                    pair1_idx = ordered_delivery_of_pairs[0];
                    pair1_data = pairs_with_assigned_targets[pair1_idx];
                    cost_trip1, end_pos_trip1, trip_desc1 = calculate_trip_cost(current_robot_loc_3c,
                                                                                [(pair1_data[0], pair1_data[1])],
                                                                                [pair1_data[2]]);
                    if cost_trip1 == float('inf'):
                        is_valid_3c = False;
                    else:
                        current_cost_3c += cost_trip1;
                        current_robot_loc_3c = end_pos_trip1;
                        current_path_3c.extend(trip_desc1);

                    # Trip 2: 1-pair (second pair)
                    if is_valid_3c:
                        pair2_idx = ordered_delivery_of_pairs[1];
                        pair2_data = pairs_with_assigned_targets[pair2_idx];
                        cost_trip2, end_pos_trip2, trip_desc2 = calculate_trip_cost(current_robot_loc_3c,
                                                                                    [(pair2_data[0], pair2_data[1])],
                                                                                    [pair2_data[2]]);
                        if cost_trip2 == float('inf'):
                            is_valid_3c = False;
                        else:
                            current_cost_3c += cost_trip2;
                            current_robot_loc_3c = end_pos_trip2;
                            current_path_3c.extend(trip_desc2);

                    # Trip 3: 2-pair (last two pairs)
                    if is_valid_3c:
                        pair3_idx = ordered_delivery_of_pairs[2];
                        pair4_idx = ordered_delivery_of_pairs[3];
                        pair3_data = pairs_with_assigned_targets[pair3_idx];
                        pair4_data = pairs_with_assigned_targets[pair4_idx];
                        cost_trip3, end_pos_trip3, trip_desc3 = calculate_trip_cost(current_robot_loc_3c,
                                                                                    [(pair3_data[0], pair3_data[1]), (pair4_data[0], pair4_data[1])],
                                                                                    [pair3_data[2], pair4_data[2]]);
                        if cost_trip3 == float('inf'):
                            is_valid_3c = False;
                        else:
                            current_cost_3c += cost_trip3;
                            current_robot_loc_3c = end_pos_trip3;
                            current_path_3c.extend(trip_desc3);

                    if is_valid_3c and current_cost_3c < min_total_distance:
                        min_total_distance = current_cost_3c;
                        best_overall_path_sequence = current_path_3c;
                        # print(f"New best (1x1 + 1x1 + 1x2 delivery): {min_total_distance}")

    # --- End timing the route calculation ---
    end_time = time.time();
    duration = end_time - start_time;

    print("\n--- Final Result ---");
    print(f"Minimum total distance: {min_total_distance}");
    print("Optimal Path Sequence:");
    for step in best_overall_path_sequence:
        print(f"- {step}");
    print(f"\nTime taken to calculate optimal route: {duration:.4f} seconds");

# --- Run the simulation ---
if __name__ == "__main__":
    solve_robot_challenge_with_red_zones();4
