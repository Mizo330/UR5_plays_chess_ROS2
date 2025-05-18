#Turn
bool WHITE = True
bool BLACK = False

#Specialmove
int16 SPECIAL_MOVE_NONE = 0
int16 SPECIAL_MOVE_CAPTURE = 1
int16 SPECIAL_MOVE_KINGSIDE_CASTLE = 2
int16 SPECIAL_MOVE_QUEENSIDE_CASTLE = 3
int16 SPECIAL_MOVE_EN_PASSANT = 4

string move_uci
bool is_promotion
bool is_check
bool is_checkmate
bool turn 
int16 special_move
string moved_piece
int16 move_number
int16 ply