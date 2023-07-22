package org.firstinspires.ftc.teamcode.commandBased.classes.util;

import java.util.ArrayList;
import java.util.List;

public class Matrix {
    double[][] data;
    int rows, cols;

    //word I don't remember (:
    public Matrix(int rows, int cols) {
        data = new double[rows][cols];
        this.rows = rows;
        this.cols = cols;
        for (int i = 0; i < rows; i++) {
            for ( int j = 0; j < cols; j++) {
                data[i][j]=0;
            }
        }
    }


    //Static functions
    public static void printMatrix(Matrix a) {
        for (int i = 0; i < a.rows; i++) {
            for (int j = 0; j < a.cols; j++) {
                System.out.print(a.data[i][j] + "\t");
            }
            System.out.println();
        }
    }

    public static Matrix add(Matrix a, Matrix b) {
        Matrix temp = new Matrix(a.cols, a.rows);
        for (int i = 0; i < a.rows; i++) {
            for (int j = 0; j < a.cols; j++) {
                temp.data[i][j]=a.data[i][j] + b.data[i][j];
            }
        }
        return temp;
    }

    public static Matrix subtract(Matrix a, Matrix b) {
        Matrix temp = new Matrix(a.cols, a.rows);
        for (int i = 0; i < a.rows; i++) {
            for (int j = 0; j < a.cols; j++) {
                temp.data[i][j]=a.data[i][j] - b.data[i][j];
            }
        }
        return temp;
    }

    public static Matrix transpose(Matrix a) {
        Matrix temp = new Matrix(a.cols, a.rows);
        for (int i = 0; i < a.rows; i++) {
            for (int j = 0; j < a.cols; j++) {
                temp.data[j][i] = a.data[i][j];
            }
        }
        return temp;
    }

    public static Matrix dotProduct(Matrix a, Matrix b) {
        Matrix temp = new Matrix(a.rows, b.cols);
        for (int i = 0; i < temp.rows; i++) {
            for (int j = 0; j < temp.cols; j++) {
                double sum = 0;
                for (int k = 0; k < a.cols; k++) {
                    sum += a.data[i][k]*b.data[k][j];
                }
                temp.data[i][j] = sum;
            }
        }
        return temp;
    }

    public static Matrix fromArray(double[] x) {
        Matrix temp = new Matrix(x.length, 1);
        for (int i = 0; i < x.length; i++) {
            temp.data[i][0]=x[i];
        }
        return temp;
    }


    //Addition functions
    public void add(double scalar) {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                this.data[i][j] += scalar;
            }
        }
    }

    public void add(Matrix m) {
        if (cols != m.cols || rows != m.rows) {
            System.out.println("Shape does not match");
            return;
        }
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                this.data[i][j] += m.data[i][j];
            }
        }
    }


    //Subtraction functions
    public void subtract(double scalar) {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                this.data[i][j] -= scalar;
            }
        }
    }

    public void subtract(Matrix m) {
        if (cols != m.cols || rows != m.rows) {
            System.out.println("Shape does not match");
            return;
        }
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                this.data[i][j] -= m.data[i][j];
            }
        }
    }


    //Multiplication functions
    public void multiply(double scalar) {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                this.data[i][j] *= scalar;
            }
        }
    }

    public void multiply(Matrix m) {
        if (cols != m.cols || rows != m.rows) {
            System.out.println("Shape does not match");
            return;
        }
        for (int i = 0; i < m.rows; i++) {
            for (int j = 0; j < m.cols; j++) {
                this.data[i][j] *= m.data[i][j];
            }
        }
    }


    //Sigmoid functions
    public void sigmoid() {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                this.data[i][j] = 1/(1+Math.exp(-this.data[i][j]));
            }
        }
    }

    public Matrix dsigmoid() {
        Matrix temp = new Matrix(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                temp.data[i][j] = this.data[i][j] * (1-this.data[i][j]);
            }
        }
        return temp;
    }

    public double get(int rowNumber, int columnNumber) {
        return data[rowNumber][columnNumber];
    }


    //array transformations
    public List<Double> toArray() {
        List<Double> temp = new ArrayList<Double>();
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                temp.add(data[i][j]);
            }
        }
        return temp;
    }
}
