/*
CLASS TO IMPLEMENT THE TICTACTOE GAMEPLAY LOGIC

Used in brain.cpp
*/
#include <algorithm>
#include <limits>
#include <array>

class TicTacToe {
public:
    static constexpr int EMPTY = 2;
    static constexpr int WHITE = 1; // Computer
    static constexpr int BLACK = 0; // Human
    static constexpr double CELL_SIZE = 0.1;

    TicTacToe() {
        resetBoard();
    }

    // Reset the board to all EMPTY
    void resetBoard() {
        board_.fill(EMPTY);
    }

    // Place a move
    bool makeMove(int index, int player) {
        if (index < 0 || index >= 9 || board_[index] != EMPTY)
            return false;
        board_[index] = player;
        return true;
    }

    // Find best move for AI
    int findBestMove() {
        int bestVal = std::numeric_limits<int>::min();
        int bestMove = -1;

        for (int i = 0; i < 9; i++) {
            if (board_[i] == EMPTY) {
                board_[i] = WHITE;
                int moveVal = minimax(0, false);
                board_[i] = EMPTY;

                if (moveVal > bestVal) {
                    bestMove = i;
                    bestVal = moveVal;
                }
            }
        }
        return bestMove;
    }

    // Check if someone has won
    bool isGameOver() {
        return evaluate() != 0 || !isMovesLeft();
    }

    // Return board for visualization/debugging
    const std::array<int, 9>& getBoard() const { return board_; }

private:
    std::array<int, 9> board_;

    bool isMovesLeft() const {
        return std::any_of(board_.begin(), board_.end(), [](int c){ return c == EMPTY; });
    }

    int evaluate() const {
        static const int winCombos[8][3] = {
            {0,1,2},{3,4,5},{6,7,8},
            {0,3,6},{1,4,7},{2,5,8},
            {0,4,8},{2,4,6}
        };

        for (auto &combo : winCombos) {
            if (board_[combo[0]] == board_[combo[1]] &&
                board_[combo[1]] == board_[combo[2]] &&
                board_[combo[0]] != EMPTY) {
                if (board_[combo[0]] == WHITE) return 10;
                else if (board_[combo[0]] == BLACK) return -10;
            }
        }
        return 0;
    }

    int minimax(int depth, bool isMaximizingPlayer) {
        int score = evaluate();
        if (score == 10) return score - depth;
        if (score == -10) return score + depth;
        if (!isMovesLeft()) return 0;

        if (isMaximizingPlayer) {
            int best = std::numeric_limits<int>::min();
            for (int i = 0; i < 9; i++) {
                if (board_[i] == EMPTY) {
                    board_[i] = WHITE;
                    best = std::max(best, minimax(depth + 1, false));
                    board_[i] = EMPTY;
                }
            }
            return best;
        } else {
            int best = std::numeric_limits<int>::max();
            for (int i = 0; i < 9; i++) {
                if (board_[i] == EMPTY) {
                    board_[i] = BLACK;
                    best = std::min(best, minimax(depth + 1, true));
                    board_[i] = EMPTY;
                }
            }
            return best;
        }
    }
};
