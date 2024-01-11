import numpy as np
import math
from itertools import zip_longest

class poly:
    def __init__(self, coefficients):
        self.inim(coefficients)

    def inim(self, coefficients):
        # Ensure the coefficients are in the correct order (from the highest degree to the constant term)
        self.coefficients = []
        if len(coefficients)==1 and abs(coefficients[0])<1e-7:
            self.coefficients=[0]
            return
        zeros_end = False
        for i in range(len(coefficients)):
            if not zeros_end:
                if abs(coefficients[len(coefficients) - 1 - i])<1e-7:
                    continue
                else:
                    zeros_end = True
            if zeros_end:
                m =  coefficients[len(coefficients) - 1 - i]
                if abs(m)<1e-5:
                    m=0
                self.coefficients.insert(0, m)

        if not zeros_end:
            self.coefficients = [0]

    def __str__(self):
        # Convert the polynomial to a human-readable string
        terms = [f"{coeff}x^{deg}" if deg > 0 else str(coeff) for deg, coeff in enumerate(self.coefficients) if coeff != 0]
        return " + ".join(terms[::-1])

    def __add__(self, other):
        # Add two polynomials
        if self.is_zero():
            return poly(other.coefficients)
        if other.is_zero():
            return poly(self.coefficients)
        result_coeffs = [sum(x) for x in zip_longest(self.coefficients, other.coefficients, fillvalue=0)]
        return poly(result_coeffs)

    def __mul__(self, other):
        if isinstance(other, float) or isinstance(other, int):
            # Multiply each coefficient by the float value
            if not self.is_zero():
                result_coeffs = [coeff * other for coeff in self.coefficients]
            else :
                result_coeffs= [0]

        elif isinstance(other, poly):    
            # Multiply two polynomials
            if self.is_zero():
                return poly([0])
            if other.is_zero():
                return poly([0])

            result_coeffs = [0] * (len(self.coefficients) + len(other.coefficients) - 1)
            
            for i in range(len(self.coefficients)):
                for j in range(len(other.coefficients)):
                    result_coeffs[i + j] += self.coefficients[i] * other.coefficients[j]

        return poly(result_coeffs)

    def is_zero(self):
        if len(self.coefficients)==1 and abs(self.coefficients[0] )<1e-7:
            return True
        return False

    def copy(self):
        return poly(self.coefficients)

    def normalize(self):
        v = self.coefficients[-1]
        #for i in range(5):
        #    v = self.coefficients[8-i]
        #    if v!=0:
        #        break
        res = [0 for i in range(len(self.coefficients))]
        for i in range(len(self.coefficients)):
            res[i] = self.coefficients[i] /v

        self.inim(res)
        return 

    def evaluate(self, x):
        result = 0
        for deg, coeff in enumerate(self.coefficients):
            result += coeff * (x ** deg)
        return result

    def polydiv(self,num, den):
        #Create normalized copies of the args
        num = num[:]
        den = den[:]

        if len(num) >= len(den):
            #Shift den towards right so it's the same degree as num
            shiftlen = len(num) - len(den)
            den = [0] * shiftlen + den
        else:
            return [0], num

        quot = []
        divisor = float(den[-1])
        for i in range(shiftlen + 1):
            #Get the next coefficient of the quotient.
            mult = num[-1] / divisor
            quot = [mult] + quot

            #Subtract mult * den from num, but don't bother if mult == 0
            #Note that when i==0, mult!=0; so quot is automatically normalized.
            if mult != 0:
                d = [mult * u for u in den]
                num = [u - v for u, v in zip(num, d)]

            num.pop()
            den.pop(0)

        return quot, poly(num).coefficients