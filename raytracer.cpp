#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "Template_creation_image/stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "Template_creation_image/stb_image.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>

class Vector {
public:
    explicit Vector(double x=0, double y=0, double z=0) {
        coords[0]=x;
        coords[1]=y;
        coords[2]=z;
    };
    double operator[](int i) const{ return coords[i]; };
    double &operator[](int i) { return coords[i]; };
    double sqrNorm() const {
        return coords[0]*coords[0] + coords[1]*coords[1] + coords[2]*coords[2];
    };
    Vector get_normalized() const {
        double norm = sqrt(sqrNorm());
        return Vector(coords[0]/norm, coords[1]/norm, coords[2]/norm);
    };
private:
    double coords[3];
};

Vector operator+(const Vector& a, const Vector& b) {
    return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
    return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(double a, const Vector& b) {
    return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator/(const Vector& b, double a) {
    return Vector(b[0]/a, b[1]/a, b[2]/a);
}
Vector operator*(const Vector& a, const Vector& b) {
    return Vector(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}
double dot(const Vector& a, const Vector& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

class Ray{
public:
    Ray(const Vector& C, const Vector& u) : C(C), u(u) {
    }
    Vector C;
    Vector u;
};

class Sphere {
public:
    Sphere(const Vector& O, double R, Vector rho) : O(O), R(R), rho(rho) {
    }
    bool intersect(const Ray& r, Vector& P, Vector& N, double& t){
        // pour résoudre a*t^2 + b*t + c = 0
        double a = 1;
        double b = 2* dot(r.u, r.C - O);
        double c = (r.C-O).sqrNorm() - R*R;
        double delta = b*b - 4*a*c;
        if (delta<0){
            return false;
        }
        else{
            double sqrtDelta = sqrt(delta);
            double t2 = (-b + sqrtDelta)/(2*a);
            if (t2 < 0){
                return false;
            }

            double t;
            double t1 = (-b - sqrtDelta)/(2*a);
            if (t1 > 0 ){
                t = t1;
            }
            else {
                t = t2;
            }
            P = r.C + t*r.u;
            N = (P-O).get_normalized();
            return true;
        }
    }
    Vector O;
    double R;
    Vector rho;
};

class Light {
public:
    Light(const double& I, const Vector& L) : I(I), L(L) {
    }
    double I;
    Vector L;
};

class Scene{
public:
    Scene(Light& Lum, Vector& color) : Lum(Lum), color(color) {
    }
    Vector intersect_couleur(Ray& rayon){
        Vector P,N;
        double t = 2147483647;
        Vector couleur = color;
        for(Sphere& S : objects){
            Vector P_current,N_current;
            double t_current;
            bool inter = S.intersect(rayon, P_current, N_current, t_current);
            if (inter && t_current < t){
                P = P_current;
                N = N_current;
                Vector PL;
                PL = Lum.L-P;
                couleur = Lum.I/(4*M_PI*PL.sqrNorm()) * std::max(0., dot(N, PL.get_normalized())) * S.rho/M_PI;
                return couleur;
            }
        }
    }
    std::vector<Sphere> objects;
    Light Lum;
    Vector color;
};

int main() {
	int W = 512;
	int H = 512;

    Vector C(0,0,55);
    Vector O(0,0,0);
    Vector rho(1,0,0);
    int r = 10;
    Sphere S(O, r, rho);
    double fov = 60 * M_PI/180;
    Light Lum(double (1E7), Vector (-10, 20, 40));
    Vector couleur(0, 0, 0);
    //double I = 1E7;
    //Vector L(-10,20,40);

	std::vector<unsigned char> image(W*H * 3, 0);
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {

            Vector u(j-W/2, i-H/2,  -W/(2*tan(fov/2)) );
            u = u.get_normalized();
            Ray rayon(C,u);
            Vector P,N;
            double t;
            bool inter = S.intersect(rayon, P, N, t);
            Vector coul = couleur;
            if (inter){
                Vector PL;
                PL = Lum.L-P;
                coul = Lum.I/(4*M_PI*PL.sqrNorm()) * std::max(0., dot(N, PL.get_normalized())) * S.rho/M_PI;
            }

            image[( (H-i-1)*W + j) * 3 + 0] = coul[0];
            image[( (H-i-1)*W + j) * 3 + 1] = coul[1];
            image[( (H-i-1)*W + j) * 3 + 2] = coul[2];

		}
	}
	stbi_write_png("D:/julbr/Documents/ecole/ECL/3A/MOS_2.2_Informatique_Graphique/Image/image111.png", W, H, 3, &image[0], 0);

	return 0;
}
