# toh_solution_generator.py

def tower_of_hanoi_tagged_steps(n, source_rod, destination_rod, auxiliary_rod):
    """
    Generates the steps to solve the Tower of Hanoi puzzle in a tagged format.
    Format: MD<disk_number><source_rod><destination_rod>
    Example: MD1AC means "Move disk 1 from rod A to rod C"

    Args:
        n (int): Current disk number being considered (starts with the largest).
                 For the recursive calls, this represents the number of disks in the sub-problem.
        source_rod (str): Name of the source rod.
        destination_rod (str): Name of the destination rod.
        auxiliary_rod (str): Name of the auxiliary rod.

    Yields:
        str: A string describing a single move in the tagged format.
    """
    if n == 1:
        yield f"MD{n}{source_rod}{destination_rod}"
        return
    # In the standard algorithm, when we say "move n-1 disks", the actual disk numbers
    # involved are 1 to n-1. The `n` in `MDnSC` refers to the specific disk being moved.
    # The recursive function needs to know which *physical* disk it's telling to move.
    # So, the disk number in the output string should correspond to the disk being moved.

    # To achieve this, we need to track the actual disk numbers.
    # The standard recursive formulation implicitly handles this by referring to "the nth disk"
    # as the largest disk in the current subproblem.

    # Let's use a helper that keeps track of the *actual* disk numbers.
    # The `tower_of_hanoi_steps_recursive` function is a more direct translation of the
    # standard algorithm, where 'k' is the disk number (1 being smallest, num_disks being largest).

    # Simpler approach for tagged steps:
    # The `n` in the recursive call refers to the 'stack height' or 'sub-problem size'.
    # The actual disk moved is always the largest in that sub-problem.
    # For the `toh_visual.py` script, `disk_num` is the actual value on the disk (1 to num_disks).

    # Let's re-think the `n` parameter. It typically means "number of disks to move".
    # The disk being moved is `n` itself in the main step.
    if n > 0:
        # Move n-1 disks from source to auxiliary, so they are out of the way
        yield from tower_of_hanoi_tagged_steps(n - 1, source_rod, auxiliary_rod, destination_rod)

        # Move the nth disk from source to destination
        yield f"MD{n}{source_rod}{destination_rod}"

        # Move the n-1 disks from auxiliary to destination
        yield from tower_of_hanoi_tagged_steps(n - 1, auxiliary_rod, destination_rod, source_rod)


def generate_tower_of_hanoi_html_tagged(num_disks, output_filename="tower_of_hanoi_tagged_solution.html"):
    """
    Generates an HTML file with the Tower of Hanoi solution using tagged steps.

    Args:
        num_disks (int): The number of disks for the puzzle.
        output_filename (str): The name of the HTML file to create.
    """
    if not isinstance(num_disks, int) or num_disks < 1:
        print("Error: Number of disks must be a positive integer.")
        return

    # The `tower_of_hanoi_tagged_steps` expects `n` to be the largest disk initially.
    steps = list(tower_of_hanoi_tagged_steps(num_disks, 'A', 'C', 'B'))

    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Tower of Hanoi Solution ({num_disks} Disks) - Tagged Format</title>
    <style>
        body {{ font-family: sans-serif; line-height: 1.6; padding: 20px; max-width: 800px; margin: auto; }}
        h1 {{ color: #333; }}
        h2 {{ color: #555; }}
        ul {{ list-style-type: decimal; padding-left: 20px; }}
        li {{ margin-bottom: 8px; font-family: monospace; }}
        .explanation {{ background-color: #f0f0f0; border-left: 4px solid #007bff; padding: 10px; margin-bottom:20px;}}
        .footer {{ margin-top: 30px; font-size: 0.9em; color: #777; }}
    </style>
</head>
<body>
    <h1>Tower of Hanoi Solution - Tagged Format</h1>
    <h2>Problem: {num_disks} Disks</h2>
    <p>The objective of the Tower of Hanoi puzzle is to move the entire stack of disks from the source rod (A) to the destination rod (C), obeying the following simple rules:</p>
    <ul>
        <li>Only one disk can be moved at a time.</li>
        <li>Each move consists of taking the upper disk from one of the stacks and placing it on top of another stack or on an empty rod.</li>
        <li>No larger disk may be placed on top of a smaller disk (disks are numbered 1 to N, where 1 is the smallest).</li>
    </ul>

    <div class="explanation">
        <p><strong>Move Tag Format:</strong></p>
        <p>Each move is represented by a tag: <code>MD&lt;disk_number&gt;&lt;source_peg&gt;&lt;destination_peg&gt;</code></p>
        <ul>
            <li><code>M</code>: Stands for Move.</li>
            <li><code>D</code>: Stands for Disk.</li>
            <li><code>&lt;disk_number&gt;</code>: The number of the disk being moved (e.g., 1, 2, ..., {num_disks}).</li>
            <li><code>&lt;source_peg&gt;</code>: The peg the disk is moved FROM (A, B, or C).</li>
            <li><code>&lt;destination_peg&gt;</code>: The peg the disk is moved TO (A, B, or C).</li>
        </ul>
        <p>Example: <code>MD1AC</code> means "Move disk 1 from peg A to peg C".</p>
    </div>

    <h2>Solution Steps (Source: A, Destination: C, Auxiliary: B):</h2>
    <p>The disks are numbered 1 (smallest) to {num_disks} (largest).</p>
    <ul>
"""
    for i, step in enumerate(steps):
        html_content += f"        <li>{i+1}. {step}</li>\n"

    html_content += f"""    </ul>
    <p class="footer">Total moves: {len(steps)} (which is $2^{num_disks} - 1$)</p>
</body>
</html>
"""

    try:
        with open(output_filename, "w", encoding="utf-8") as f:
            f.write(html_content)
        print(f"Successfully generated '{output_filename}' with the tagged solution for {num_disks} disks.")
    except IOError as e:
        print(f"Error writing to file '{output_filename}': {e}")

if __name__ == "__main__":
    number_of_disks = 3 # You can change this
    # This should generate the same sequence as in your `toh_visual.py` example for 3 disks:
    # ["MD1AC", "MD2AB", "MD1CB", "MD3AC", "MD1BA", "MD2BC", "MD1AC"]

    generate_tower_of_hanoi_html_tagged(number_of_disks, "hanoi_tagged_solution_3_disks.html")

    # For testing with 4 disks:
    # generate_tower_of_hanoi_html_tagged(4, "hanoi_tagged_solution_4_disks.html")