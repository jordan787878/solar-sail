#include <iostream>
#include <vector>
#include <map>

class SparseMatrix {
public:
    std::vector<std::map<unsigned int, double>> matrix;
    unsigned int rows;
    unsigned int cols;
    
    // Constructor
    SparseMatrix(unsigned int numRows, unsigned int numCols) : rows(numRows), cols(numCols) {
        matrix.resize(numRows);
    }

    // Set an element at a specific position
    void setElement(unsigned int row, unsigned int col, double value) {
        if (row >= 0 && row < rows && col >= 0 && col < cols) {
            matrix[row][col] = value;
        } else {
            std::cerr << "Invalid position (" << row << ", " << col << ")" << std::endl;
        }
    }

    // Get an element at a specific position
    unsigned int getElement(unsigned int row, unsigned int col) const {
        if (row >= 0 && row < rows && col >= 0 && col < cols) {
            auto it = matrix[row].find(col);
            if (it != matrix[row].end()) {
                return it->second;
            } else {
                return 0; // Default value for sparse matrix
            }
        } else {
            std::cerr << "Invalid position (" << row << ", " << col << ")" << std::endl;
            return 0; // Default value for sparse matrix
        }
    }

    // Print non-zero elements of the matrix
    void printNonZeroElements() const {
        for (int i = 0; i < rows; ++i) {
            for (const auto& entry : matrix[i]) {
                std::cout << "(" << i << ", " << entry.first << "): " << entry.second << std::endl;
            }
        }
    }
};