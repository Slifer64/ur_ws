#ifndef AS64_IO_UTILS_H
#define AS64_IO_UTILS_H

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
 #include <conio.h>   //for getch(), needed in wait_for_key()
#endif

#include <iostream>
#include <fstream>
#include <memory>
#include <iomanip>
#include <cerrno>
#include <streambuf>

#include <unistd.h> //used for custon getch()
#include <termios.h> //used for custon getch()
#include <fcntl.h>

#include <armadillo>

namespace io_
{

/** \brief Pauses until a key is pressed.
 */
inline void wait_for_key()
{
  #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)  // every keypress registered, also arrow keys
      std::cout << std::endl << "Press any key to continue..." << std::endl;

      FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));
      _getch();
  #elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
      std::cout << std::endl << "Press ENTER to continue..." << std::endl;

      std::cin.clear();
      std::cin.ignore(std::cin.rdbuf()->in_avail());
      std::cin.get();
  #endif
      return;
}

/** \brief Returns 1 if a key on the keyboard is pressed, 0 otherwise.
 *  @return 1 if a key on the keyboard is pressed, 0 otherwise.
 */
int kbhit(void);

/** \brief Reads a character from the keyboard (no need to press 'enter' afterwards is required)
 *  @return the character pressed in the keyboard.
 */
char getch();

void print_vectorString(const std::vector<std::string> &v, std::ostream& out=std::cout, char delim=',');

/** \brief Reads a 2D matrix from stream \a in.
 *  \details Reads the elements of the matrix row by row.
 *  @param[out] m The 2D matrix
 *  @param[in] n_rows Number of rows of the matrix
 *  @param[in] n_cols Number of columns of the matrix
 *  @param[in] in The input stream (default = std::cin).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 */
void read_mat(arma::mat &m, long n_rows, long n_cols, std::istream &in = std::cin, bool binary = false);


/** \brief Reads a 2D matrix from stream \a in.
 *  \details Reads first the number of rows and columns and then the elements of the matrix row by row.
 *  @param[out] m The 2D matrix
 *  @param[in] in The input stream (default = std::cin).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 */
void read_mat(arma::mat &m, std::istream &in = std::cin, bool binary = false);

/** \brief Reads a scalar value from stream \a in.
 *  \details Reads a scalar value in the format specified by \a binary flag.
 *  @param[out] scalar scalar value.
 *  @param[in] in The input stream (default = std::cin).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 */
template <typename T>
void read_scalar(T &scalar, std::istream &in = std::cin, bool binary = false)
{
  if (binary) in.read((char *)(&scalar), sizeof(scalar));
  else in >> scalar;
}

/** \brief Writes a 2D matrix in stream 'fid'.
 *  \details Writes the elements of the matrix row by row.
 *  @param[in] m The 2D matrix
 *  @param[in] n_rows Number of rows of the matrix
 *  @param[in] n_cols Number of columns of the matrix
 *  @param[in] out The output stream (optional, default = 1 for output to screen).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 *  @param[in] precision precision in txt format (optional, default = 6).
 */
void write_mat(const arma::mat &m, int n_rows, int n_cols, std::ostream &out = std::cout, bool binary = false, int precision = 6);


/** \brief Writes a 2D matrix in stream \a out.
 *  \details Writes first the number of rows and columns and then the elements of the matrix row by row.
 *  @param[in] m The 2D matrix
 *  @param[in] out The output stream (optional, default = 1 for output to screen).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 *  @param[in] precision precision in txt format (optional, default = 6).
 */
void write_mat(const arma::mat &m, std::ostream &out = std::cout, bool binary = false, int precision = 6);

/** \brief Writes a scalar value in stream \a out
 *  \details Writes a scalar in the format specified by \a binary flag. If the format is binary,
 *           the class type of \a scalar is used to determine the number of bytes to use.
 *  @param[in] scalar scalar value.
 *  @param[in] out The output stream (optional, default = std::cout).
 *  @param[in] binary Flag indicating the format (true for binary, false for text, optional, default = false).
 *  @param[in] precision precision in txt format (optional, default = 6).
 */
template <typename T>
void write_scalar(T scalar, std::ostream &out = std::cout, bool binary = false, int precision = 6)
{
  if (binary) out.write((const char *)(&scalar), sizeof(scalar));
  else out << std::setprecision(precision) << scalar;
}


/** \brief Reads all contents of a file and returns them as a string;
 *  @param[in] filename The path+name of the file to read.
 *  @param[in] str The contents of the file as a string.
 *
 *  \throws Throws an std::ios_base::failure if it fails to read the file.
 */
void readFile(const std::string &filename, std::string &contents);

} // namespace io_


#endif // AS64_IO_UTILS_H
