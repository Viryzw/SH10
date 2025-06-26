from printer import DataPrinter

if __name__ == "__main__":

    a = 3.1415
    b = 2.718
    c = "test"
    d = 42
    printer = DataPrinter(float_precision=2, rows=2, cols=2, log_file_name="log.csv")
    while 1:
        printer.nprint(a, bi=b, c=c, d=d)

