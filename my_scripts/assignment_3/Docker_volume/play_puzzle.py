from toh_visual import TowerOfHanoi
from ollama_call import ask_llm
import time

def get_current_state_description(hanoi):
    """Generate a text description of the current state of the Tower of Hanoi"""
    state_desc = "Current Tower of Hanoi state:\n"
    for peg, disks in hanoi.pegs.items():
        disk_str = ", ".join([str(d) for d in disks]) if disks else "empty"
        state_desc += f"Peg {peg}: {disk_str}\n"
    return state_desc

def get_next_move(hanoi):
    """Ask the LLM for the next valid move based on current state"""
    state_description = get_current_state_description(hanoi)
    
    prompt = f"""
{state_description}

Please suggest a valid next move for the Tower of Hanoi puzzle.
The goal is to move all disks from peg A to peg C, following these rules:
1. Only one disk can be moved at a time
2. Only the topmost disk from a peg can be moved
3. A larger disk cannot be placed on top of a smaller disk

Return your answer in the exact format: "MDxYZ" where:
- x is the disk number to move
- Y is the source peg (A, B, or C)
- Z is the destination peg (A, B, or C)

For example: "MD1AC" means "Move Disk 1 from peg A to peg C"
"""
    
    response = ask_llm(prompt)
    print(f"LLM suggestion: {response}")
    
    # Extract the move code from the response (looking for MDxYZ pattern)
    import re
    move_pattern = re.compile(r'MD\d[ABC][ABC]')
    move_match = move_pattern.search(response)
    
    if move_match:
        return move_match.group(0)
    else:
        print("Could not extract a valid move from LLM response")
        return None

def interactive_tower_of_hanoi(num_disks=3):
    """Run an interactive Tower of Hanoi with LLM suggestions"""
    hanoi = TowerOfHanoi(num_disks)
    print(f"Starting Tower of Hanoi with {num_disks} disks")
    
    # Keep track of the moves made
    moves_made = []
    
    while len(hanoi.pegs['C']) != num_disks:
        print("\n" + get_current_state_description(hanoi))
        
        choice = input("Get LLM suggestion for next move? (y/n) or enter move directly: ")
        
        if choice.lower() == 'y':
            move = get_next_move(hanoi)
            if move:
                confirm = input(f"Apply suggested move {move}? (y/n): ")
                if confirm.lower() != 'y':
                    continue
            else:
                continue
        elif len(choice) == 5 and choice.startswith("MD"):
            move = choice
        else:
            print("Invalid input. Please try again.")
            continue
        
        # Parse and execute the move
        try:
            disk_num = int(move[2])
            from_peg = move[3]
            to_peg = move[4]
            
            if hanoi.move_disk(disk_num, from_peg, to_peg):
                moves_made.append(move)
                print(f"Moved disk {disk_num} from peg {from_peg} to peg {to_peg}")
            else:
                print("Invalid move, please try again.")
        except (IndexError, ValueError) as e:
            print(f"Error parsing move: {e}")
    
    print("\nCongratulations! You've solved the Tower of Hanoi puzzle.")
    print(f"Moves made: {len(moves_made)}")
    print(f"Moves: {', '.join(moves_made)}")
    
    input("Press Enter to close...")

if __name__ == "__main__":
    interactive_tower_of_hanoi(3)