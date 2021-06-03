#include<iostream>
#include<conio.h>
#include<math.h>
#include<iomanip>
using namespace std;
float epsilon=1e-10;
class Vector3
{
private:
    float x_comp,y_comp,z_comp;
public:
    
    Vector3(float x=0.0,float y=0.0,float z=0.0)
    {set(x,y,z);}
    
    Vector3(const Vector3& v)                       
    { setXcomp(v.getX());
      setYcomp(v.getY());
      setZcomp(v.getZ());
    }
    
    void set(float x,float y,float z)               
    {setXcomp(x);setYcomp(y);setZcomp(z);}
    
    void setXcomp(float x)                           
    {x_comp=x;}

    void setYcomp(float y)
    {y_comp=y;}

    void setZcomp(float z)
    {z_comp=z;}

    float getX() const     
    {return x_comp;} 

    float getY() const
    {return y_comp;}

    float getZ() const
    {return z_comp;}
    
    float Magnitude()
    {float magSquare=pow(getX(),2)+pow(getY(),2)+pow(getZ(),2); return sqrt(magSquare);}

    bool isUnitVector3()  // float should not be compared,we will  
    {return abs(Magnitude()-1.0)<epsilon;}

    Vector3 Conjugate()
    {   
        Vector3 temp(-1*getX(),-1*getY(),-1*getZ());
        return temp;
    }

    Vector3 operator+(const Vector3& v2)
    {
        Vector3 temp;
        temp.setXcomp(getX()+v2.getX()); temp.setYcomp(getY()+v2.getY()); temp.setZcomp(getZ()+v2.getZ());
        return temp;
    }

    Vector3 operator-(const Vector3& v2)
    {
        Vector3 temp;
        temp.setXcomp(getX()-v2.getX()); temp.setYcomp(getY()-v2.getY()); temp.setZcomp(getZ()-v2.getZ());
        return temp;
    }

    Vector3 operator*(const float& factor)  //scaler multiply
    {
        Vector3 temp;
        temp.setXcomp(getX()*factor); temp.setYcomp(getY()*factor); temp.setZcomp(getZ()*factor);
        return temp;
    }

    Vector3 operator/(const float& factor)
    {
        try
        {            
            Vector3 temp;
            temp.setXcomp(getX()/factor); temp.setYcomp(getY()/factor); temp.setZcomp(getZ()/factor);
            return temp;
        }
        catch(const std::exception& e)
        {
            cout<<"Terminating Program [ERROR MESSAGE]: ZERODIVISIONERROR";
            exit(2);
        }

    }

    void showVector3(int precision=2) const
    {cout<<setprecision(precision)<<"Vector3("<<getX()<<","<<getY()<<","<<getZ()<<")"<<endl;}
    
    ~Vector3(){};
};

class Quaternion
{
private:
    float scalerPart;
    Vector3 vectorPart;
public:

    Quaternion()
    {setScaler(0);}

    Quaternion(Vector3 vector,float scaler=0.0)
    { setScaler(scaler); setVector(vector);}

    void setVector(Vector3& v)
    {vectorPart=Vector3(v);}

    void setScaler(float scaler)
    {scalerPart=scaler;}
        
    float getScalerPart()   const
    {return scalerPart;}

    Vector3 getVectorPart() const
    {return vectorPart;}

    float Norm()
    {
        float normSquare=pow(getScalerPart(),2)+pow(getVectorPart().getX(),2)+pow(getVectorPart().getY(),2)+pow(getVectorPart().getZ(),2);
        return sqrt(normSquare);
    }
    Quaternion Versor()
    {
        try
        {
            Quaternion temp;
            Vector3 v=getVectorPart()/Norm();
            temp.setScaler(getScalerPart()/Norm());
            temp.setVector(v);
            return temp;
        }
        catch(const std::exception& e)
        {
            cout<<"Terminating Program [ERROR MESSAGE]: ZERODIVISIONERROR";
            exit(2);
        }    
    }
    bool isUnitQuaternion()
    {return abs(Norm()-1.0)<epsilon;}

    bool isPureQuaternion()
    {return abs(getScalerPart()-0.0)<epsilon;}
    
    Quaternion Conjugate()
    {
        Quaternion temp;
        temp.setScaler(getScalerPart());
        Vector3 vect=getVectorPart().Conjugate(); 
        temp.setVector(vect);
        return temp;
    }
    Quaternion Reciprocal()   // this is also the inverseQuaternion.
    {
        float normSquare=pow(Norm(),2);
        return Conjugate()/normSquare;
    }
    Quaternion operator+(const Quaternion& Q )
    {
        Quaternion temp;
        temp.setScaler(getScalerPart()+Q.getScalerPart());
        Vector3 vect=getVectorPart()+Q.getScalerPart();
        temp.setVector(vect);
        return temp;
    }  
    Quaternion operator-(const Quaternion& Q )
    {
        Quaternion temp;
        temp.setScaler(getScalerPart()-Q.getScalerPart());
        Vector3 vect=getVectorPart()-Q.getScalerPart();
        temp.setVector(vect);
        return temp;
    }
    //For Quaternion multiplication: (i**2= j**2 = k**2 = ijk = -1, ij=k ,jk=i ,ki=j ,ji= -k ,kj= -i ,ik= -j)
    Quaternion operator*(const Quaternion& Q )
    {
        Quaternion temp;
        float scaler   =(getScalerPart()*Q.getScalerPart())        - (getVectorPart().getX()*Q.getVectorPart().getX()) - (getVectorPart().getY()*Q.getVectorPart().getY()) - (getVectorPart().getZ()*Q.getVectorPart().getZ());  
        float vectPart1=(getVectorPart().getX()*Q.getScalerPart()) + (getScalerPart()*Q.getVectorPart().getX())        + (getVectorPart().getY()*Q.getVectorPart().getZ()) - (getVectorPart().getZ()*Q.getVectorPart().getY());     
        float vectPart2=(getScalerPart()*Q.getVectorPart().getY()) + (getVectorPart().getY()*Q.getScalerPart())        + (getVectorPart().getZ()*Q.getVectorPart().getX()) - (getVectorPart().getX()*Q.getVectorPart().getZ());
        float vectPart3=(getScalerPart()*Q.getVectorPart().getZ()) + (getVectorPart().getZ()*Q.getScalerPart())        + (getVectorPart().getX()*Q.getVectorPart().getY()) - (getVectorPart().getY()*Q.getVectorPart().getX());
        Vector3 v(vectPart1,vectPart2,vectPart3);
        temp.setScaler(scaler);
        temp.setVector(v);
        return temp;
    }

    // Scaler multiplication of Quaternion.
    Quaternion operator*(const float& F)
    {
        Quaternion temp;
        temp.setScaler(getScalerPart()*F);
        Vector3 vect=getVectorPart()*F;
        temp.setVector(vect);
        return temp;
    }

    Quaternion operator/(const float& F)
    {
        try
        {
            Quaternion temp;
            temp.setScaler(getScalerPart()/F);
            Vector3 vect=getVectorPart()/F;
            temp.setVector(vect); 
            return temp;
        }
        catch(const std::exception& e)
        {
            cout<<"Terminating Program [ERROR MESSAGE]: ZERODIVISIONERROR";
            exit(2);            
        }
    }
    void showQuaternion(int precision=4)
    {cout<<setprecision(precision)<<"Quaternion("<<getScalerPart()<<","<<"Vector3("<<getVectorPart().getX()<<","<< getVectorPart().getY()<<","<< getVectorPart().getZ()<<"))"<<endl;}
    
    ~Quaternion(){};
};

int main(int argc, char const *argv[])
{
    // Object Instantiation.
    Quaternion q1,q2(Vector3(0,9,2),1);
    Vector3 v2(12,31,11);
    q1.setScaler(12);
    q1.setVector(v2);
    q2.showQuaternion();
    (v2*3).showVector3();
    return 0;
}
