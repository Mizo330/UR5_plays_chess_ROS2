import math

corner_a1 = [0,-0.3,0.3]
board_orientation = [-1,-1]
tile_size = 0.0375

def square_to_xyz(square: str):
    # square: 'a1'..'h8'
    file = ord(square[0]) - ord('a')
    rank = int(square[1]) - 1
    # center offset: add half tile
    x = corner_a1[0] + board_orientation[0]*(file * tile_size + tile_size/2)
    y = corner_a1[1] + board_orientation[1]*(rank * tile_size + tile_size/2)
    # z could be constant above board
    z = corner_a1[2] + tile_size*1.6
    return round(x, 3), round(y, 3), round(z, 3)

def xy_to_square( x: float, y: float) -> str:
    dx = x - corner_a1[0]
    dy = y - corner_a1[1]

    file_pos = dx / tile_size if board_orientation[0] > 0 else -dx / tile_size
    rank_pos = dy / tile_size if board_orientation[1] > 0 else -dy / tile_size
    file_pos += 0.3
    file = math.floor(file_pos)
    rank = math.floor(rank_pos)
    file_char = chr(ord('a') + file)
    square = f"{file_char}{rank + 1}"
    return square

pos = square_to_xyz('`4')
sq = xy_to_square(pos[0],pos[1])
print(sq)