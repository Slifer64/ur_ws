#include <io_lib/io_utils.h>

namespace io_
{

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

char getch()
{
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
            perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror ("tcsetattr ~ICANON");
    return (buf);
}

void print_vectorString(const std::vector<std::string> &v, std::ostream& out, char delim)
{
  out << "[";
  for (int i=0;i<v.size()-1;i++) out << v[i] << delim;
  out << v.back() << "]";
}

void read_mat(arma::mat &m, long n_rows, long n_cols, std::istream &in, bool binary)
{
  m.resize(n_rows,n_cols);

  if (binary)
  {
    double *buff = new double[n_rows*n_cols];
    in.read((char *)(buff), n_rows*n_cols*sizeof(double));

    int k=0;
    for (int i=0;i<n_rows;i++)
    {
      for (int j=0;j<n_cols;j++) m(i,j) = buff[k++];
    }

    delete []buff;
  }
  else
  {
    for (int i=0;i<n_rows;i++)
    {
      for (int j=0;j<n_cols;j++) in >> m(i,j);
    }
  }

}

void read_mat(arma::mat &m, std::istream &in, bool binary)
{
  long n_rows;
  long n_cols;

  read_scalar(n_rows, in, binary);
  read_scalar(n_cols, in, binary);

  read_mat(m, n_rows, n_cols, in, binary);
}


void write_mat(const arma::mat &m, int n_rows, int n_cols, std::ostream &out, bool binary, int precision)
{
  if (binary)
  {
    double *buff = new double[n_rows*n_cols];

     int k=0;
     for (int i=0;i<n_rows;i++){
       for (int j=0;j<n_cols;j++) buff[k++] = m(i,j);
     }
     out.write((const char *)(buff), n_rows*n_cols*sizeof(double));

     delete []buff;
  }
  else
  {
    for (int i=0;i<n_rows;i++)
    {
      for (int j=0;j<n_cols;j++) out << std::setprecision(precision) << m(i,j) << " ";
      out << "\n";
    }
  }

}

void write_mat(const arma::mat &m, std::ostream &out, bool binary, int precision)
{
  long n_rows = m.n_rows;
  long n_cols = m.n_cols;

  write_scalar(n_rows, out, binary);
  if (!binary) out << "\n";
  write_scalar(n_cols, out, binary);
  if (!binary) out << "\n";

  write_mat(m, n_rows, n_cols, out, binary, precision);
}

void readFile(const std::string &filename, std::string &contents)
{
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (!in) throw std::ios_base::failure("Failed to open: \"" + filename + "\n");

  in.seekg(0, std::ios::end);
  contents.resize(in.tellg());
  in.seekg(0, std::ios::beg);
  in.read(&contents[0], contents.size());
  in.close();

  // std::ifstream in(filename, std::ios::in | std::ios::binary);
  // if (in)
  // {
  //   return(std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>()));
  // }
  // throw(errno);

  // std::ifstream in(filename, std::ios::in | std::ios::binary);
  // if (in)
  // {
  //   std::ostringstream contents;
  //   contents << in.rdbuf();
  //   in.close();
  //   return(contents.str());
  // }
  // throw(errno);
}


} // namespace io_
