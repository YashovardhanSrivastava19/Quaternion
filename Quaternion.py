#Quaternion Class in Python.
import math
import sys
class Vector3:
    def __init__(self,x_comp=0,y_comp=0,z_comp=0) -> None:
        """[Initialize Vector Components]"""
        self._x=x_comp;self._y=y_comp;self._z=z_comp

    def Magnitude(self):
        """[Returns the magnitude of the vector]"""
        mag=math.sqrt((self._x**2)+(self._y**2)+(self._z**2))
        return round(mag,Quaternion.prec)

    def Conjugate(self):
        """[Returns the conjugate of the Vector]"""
        return Vector3(-1*self._x,-1*self._y,-1*self._z)    
    
    def ToUnitVector(self):
        """[Return a unit vector(Direction) from the current vector object]"""
        return self/(self.Magnitude())

    def __add__(self,other):
        """[Add two vectors and return their result(new vector)]"""
        return Vector3((self._x+other._x),(self._y+other._y),(self._z+other._z))

    def __sub__(self,other):
        """[Subtract two vectors and return their result(new vector)]"""
        return Vector3((self._x-other._x),(self._y-other._y),(self._z-other._z))

    def __mul__(self,factor):
        """[Scaler multiply two vectors and return their result(new vector)]"""
        return Vector3((self._x*factor),(self._y*factor),(self._z*factor))

    def __truediv__(self,factor):
        """[Scaler divide two vectors and return their result(new vector)]"""
        try:
            v1=self._x/factor;v2=self._y/factor;v3=self._z/factor
            return Vector3(round(v1,Quaternion.prec),round(v2,Quaternion.prec),round(v3,Quaternion.prec))
        except ZeroDivisionError as _:
            print("Division by Zero not possible")
            sys.exit("[ExitInfo]:ZeroDivisionError")
      
    def __repr__(self) -> str:
        """[Represent vector object]"""
        return "Vector3({},{},{})".format(self._x,self._y,self._z)

class Quaternion:
    prec=4
    def __init__(self,scalerPart=0,vector3Comp=Vector3()) -> None:
        """[Intialize Quaternion inputs]"""
        self._a=scalerPart; self._vector=vector3Comp

    def DotProduct(self,other:"Quaternion")->float:
        """[returns the dot product of self with other Quaternion]"""
        return (self._a*other._a)+(self._vector._x*other._vector._x)+(self._vector._y*other._vector._y)+(self._vector._z*other._vector._z)

    def AngularDifference(self,other:"Quaternion")->float:
        """[returns the angular difference(in radians) between two Quaternions]"""
        return math.acos(self.DotProduct(other)/(self.Norm()*other.Norm()))
    
    def isUnitQuaternion(self)->bool:
        """[Check wheather a quaternion has unit norm or not]"""
        return math.isclose(self.Norm(),1.0)

    def Versor(self)->"Quaternion":
        """[Returns a unit Quaternion from self(Quaternion Normalization)]"""
        if math.isclose(self.Norm(),0.0):
            print("Versor does not exist for the Quaternion:{}".format(self))
            sys.exit("[INPUT ERROR]Exit_Message:ZeroNormError")

        sPart= round((self._a/self.Norm()),Quaternion.prec); vect1=self._vector/self.Norm()
        return Quaternion(sPart,vect1)
    
    def Reciprocal(self)->"Quaternion":
        """[Returns the reciprocal of the current Quaternion]"""
        return self.Conjugate()/(self.Norm()**2) 
    
    def Norm(self)->float:
        """[Returns the Norm(Magnitude) of the Quaternion]"""
        norm=math.sqrt(((self._a)**2)+((self._vector._x)**2)+((self._vector._y)**2)+((self._vector._z)**2))
        return round(norm,Quaternion.prec)

    def vectorPart(self):
        """[Returns the tuple of vector part of the Quaternion]"""
        return (self._vector)

    def scalerPart(self):
        """[Returns tuple containing scaler part of the Quaternion]"""    
        return (self._a,)
    
    def InverseQuaternion(self):
        """[returns the inverse quaternion with respect to self]"""
        return self.Conjugate()/(self.Norm()**2)
    
    def Conjugate(self):
        """[Returns the Conjugate of the Quaternion]"""        
        return Quaternion(self._a,self._vector.Conjugate())
    
    def isScaler(self)->bool:
        """[Check wheather the Quaternion is scaler(real) or not]"""    
        return self._vector._x==0 and self._vector._y==0 and self._vector._z==0 
        
    def isPure(self):
        """[Check wheather the Quaternion is pure(scaler term 0) or not]"""
        return self._a==0
    
    def __add__(self,other:"Quaternion")->"Quaternion":
        """[Add two Quaternions and return their result]"""
        if isinstance(other,Quaternion):
            sPart=self._a+other._a
            vPart=self._vector+other._vector
            print(self._vector,other._vector,vPart)
            return Quaternion(sPart,vPart)
        else:
            raise Exception("Incompatible type(s) for addition with Quaterion")

    def __sub__(self,other:"Quaternion")->"Quaternion":
        """[Subtract two Quaternions and return their result]"""
        if isinstance(other,Quaternion):
            return Quaternion(round(self._a-other._a),(self._vector-other._vector))
        else:
            raise Exception("Incompatible type(s) for subtraction with Quaterion")

    def __mul__(self,other:"Quaternion")->"Quaternion":
        #For scaler multiplication
        if isinstance(other,int) or isinstance(other,float) :
            return Quaternion((self._a*other),self._vector*other)
        
        #For Quaternion multiplication: (i**2= j**2 = k**2 = ijk = -1, ij=k ,jk=i ,ki=j ,ji= -k ,kj= -i ,ik= -j ) 
        elif isinstance(other,Quaternion):
            scalerPart= round((self._a*other._a) -(self._vector._x*other._vector._x) - (self._vector._y*other._vector._y) - (self._vector._z*other._vector._z))
            vectorPart1=round((self._vector._x*other._a) + (self._a*other._vector._x) + (self._vector._y*other._vector._z) - (self._vector._z*other._vector._y))
            vectorPart2=round((self._a*other._vector._y) + (self._vector._y*other._a) + (self._vector._z*other._vector._x) - (self._vector._x*other._vector._z))
            vectorPart3=round((self._a*other._vector._z) + (self._vector._z*other._a) + (self._vector._x*other._vector._y) - (self._vector._y*other._vector._x))
            return Quaternion(scalerPart,Vector3(vectorPart1,vectorPart2,vectorPart3))             

        else:
            raise Exception("Incompatible type(s) for multiplication with Quaterion")         
    
    def __truediv__(self,factor:float)->"Quaternion":
        """[divides each element ]"""
        try:
            a=round((self._a/factor),Quaternion.prec);
            vect=self._vector/factor
            return Quaternion(a,vect)
        except ZeroDivisionError as _:  
            print("Cannot divide by zero")
            sys.exit("[INPUT ERROR]Exit_Message:ZeroDivisionError")    
    
    def __repr__(self) -> str:
        """[represent the class object]""" 
        return "Quaternion({},{})".format(self._a,self._vector)    

#Object Instantiation
vector=Vector3(12,21,40)
quaternion=Quaternion(3,vector)
