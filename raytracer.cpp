#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "Template_creation_image/stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "Template_creation_image/stb_image.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>

#include <chrono>

#include <iostream>

class Vector {
public:
	explicit Vector(double x = 0, double y = 0, double z = 0) {
		coords[0] = x;
		coords[1] = y;
		coords[2] = z;
	};
	double operator[](int i) const { return coords[i]; };
	double& operator[](int i) { return coords[i]; };
	double sqrNorm() const {
		return coords[0] * coords[0] + coords[1] * coords[1] + coords[2] * coords[2];
	};
	Vector get_normalized() const {
		double norm = sqrt(sqrNorm());
		return Vector(coords[0] / norm, coords[1] / norm, coords[2] / norm);
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
	return Vector(a * b[0], a * b[1], a * b[2]);
}
Vector operator/(const Vector& b, double a) {
	return Vector(b[0] / a, b[1] / a, b[2] / a);
}
Vector operator*(const Vector& a, const Vector& b) {
	return Vector(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}
double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

class Ray {
public:
	Ray(const Vector& C, const Vector& u) : C(C), u(u) {
	}
	Vector C;
	Vector u;
};

class Sphere {
public:
	Sphere(const Vector& O, double R, Vector rho, bool diffuse) : O(O), R(R), rho(rho), diffuse(diffuse) {
	}
	bool intersect(const Ray& r, Vector& P, Vector& N, double& t) {
		// pour resoudre a*t^2 + b*t + c = 0
		double a = 1;
		double b = 2 * dot(r.u, r.C - O);
		double c = (r.C - O).sqrNorm() - R * R;
		double delta = b * b - 4 * a * c;
		if (delta < 0) {
			double t = 1E10;
			return false;
		}
		else {
			double sqrtDelta = sqrt(delta);
			double t2 = (-b + sqrtDelta) / (2 * a);
			if (t2 < 0) {
				return false;
			}

			double t1 = (-b - sqrtDelta) / (2 * a);
			if (t1 > 0) {
				t = t1;
			}
			else {
				t = t2;
			}
			P = r.C + t * r.u;
			N = (P - O).get_normalized();
			return true;
		}
	}
	Vector O;
	double R;
	Vector rho;
	bool diffuse;
};

class Light {
public:
	Light(const double& I, const Vector& L) : I(I), L(L) {
	}
	double I;
	Vector L;
};

class Scene {
public:
	Scene(Light& Lum, Vector& color) : Lum(Lum), color(color) {
	}
	bool intersect_scene(const Ray& rayon, Vector& P, Vector& N, Vector& albedo, double& t, bool& diffuse) {
		//Vector P, N;
		//double t = 2147483647;
		//Vector couleur = color;
		bool bool_sortie = false;
		for (Sphere& S : objects) {
			Vector P_current, N_current;
			double t_current;
			bool inter = S.intersect(rayon, P_current, N_current, t_current);
			if (inter && (t_current < t)) {
				P = P_current;
				N = N_current;
				t = t_current;
				albedo = S.rho;
				diffuse = S.diffuse;
				bool_sortie = true;
				//Vector PL;
				//PL = Lum.L - P;
				//couleur = Lum.I / (4 * M_PI * PL.sqrNorm()) * std::max(0., dot(N, PL.get_normalized())) * S.rho / M_PI;
			}
		}
		return bool_sortie;
	}
	Vector getColor(Ray& rayon, const int& rebond) {

		if (rebond > 5) return Vector(0., 0., 0.);

		Vector coul = color;
		Vector P, N, albedo;
		double t = 2147483647;
		bool diffuse;
		bool inter = intersect_scene(rayon, P, N, albedo, t, diffuse);
		if (inter) {
			Vector PL;
			PL = Lum.L - P;
			double epsilon = 0.001;
			if (diffuse) {
				Vector reflex = rayon.u - 2 * dot(rayon.u, N) * N;
				Ray rayon_reflechi(P + epsilon * reflex, reflex.get_normalized());
				return getColor( rayon_reflechi, rebond + 1);
			}
			else {
				Vector shadowP, shadowN, shadowAlbedo;
				double shadowt = 2147483647;
				bool shadowDiffuse;
				Ray shadowRay(P + epsilon * PL.get_normalized(), PL.get_normalized());
				bool shadowInter = intersect_scene(shadowRay, shadowP, shadowN, shadowAlbedo, shadowt, shadowDiffuse);
				if (shadowInter && shadowt < sqrt(PL.sqrNorm())) {
					coul = Vector(0., 0., 0.);
				}
				else {
					coul = Lum.I / (4 * M_PI * PL.sqrNorm()) * std::max(0., dot(N, PL.get_normalized())) * albedo / M_PI;
				}
			}


		}
		return coul;
	}

	std::vector<Sphere> objects;
	Light Lum;
	const Vector color;
};

int main() {
	//auto start = std::chrono::high_resolution_clock::now();
	int W = 512;
	int H = 512;

	Vector C(0, 0, 55);
	int r = 10;
	//Sphere S1(Vector(0, 0, 0), r, Vector(1, 0, 0), true);
	Sphere S1(Vector(-15, 0, 0), r, Vector(1, 0, 0), true);
	Sphere S2(Vector(15, 0, 0), r, Vector(0, 1, 0), true);
	Sphere S3(Vector(0, 15, 0), r, Vector(0, 0, 1), false);
	Sphere S4(Vector(0, -15, 0), r, Vector(0, 1, 1), true);
	Sphere SMurFace(Vector(0, 0, -1000), 940, Vector(0, 1, 0), false);
	Sphere SMurDos(Vector(0, 0, 1000), 940, Vector(1, 0, 1), false);
	Sphere SMurHaut(Vector(0, 1000, 0), 940, Vector(1, 0, 0), false);
	Sphere SMurBas(Vector(0, -1000, 0), 990, Vector(0, 0, 1), false);
	Sphere SMurDroite(Vector(1000, 0, 0), 940, Vector(0, 1, 1), false);
	Sphere SMurGauche(Vector(-1000, 0, 0), 940, Vector(1, 1, 0), false);

	double fov = 60 * M_PI / 180;
	Light Lum(double(4E9), Vector(-10, 20, 40));
	Vector couleur(0, 0, 0);
	Scene scene(Lum, couleur);
	scene.objects.push_back(S1);
	scene.objects.push_back(S2);
	scene.objects.push_back(S3);
	scene.objects.push_back(S4);
	double gamma = 0.45;
	
	scene.objects.push_back(SMurFace);
	scene.objects.push_back(SMurDos);
	scene.objects.push_back(SMurHaut);
	scene.objects.push_back(SMurBas);
	scene.objects.push_back(SMurDroite);
	scene.objects.push_back(SMurGauche);

	std::vector<unsigned char> image(W * H * 3, 0);
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {

			Vector u(j - W / 2, i - H / 2, -W / (2 * tan(fov / 2)));
			u = u.get_normalized();
			Ray rayon(C, u);
			
			Vector coul = scene.getColor(rayon, int(0));

			image[((H - i - 1) * W + j) * 3 + 0] = std::min(255., std::pow(coul[0], gamma) );
			image[((H - i - 1) * W + j) * 3 + 1] = std::min(255., std::pow(coul[1], gamma) );
			image[((H - i - 1) * W + j) * 3 + 2] = std::min(255., std::pow(coul[2], gamma) );

		}
	}
	std::string Path = "D:/julbr/Documents/ecole/ECL/3A/MOS_2.2_Informatique_Graphique/Image/";
	stbi_write_png("D:/julbr/Documents/ecole/ECL/3A/MOS_2.2_Informatique_Graphique/Image/image111.png", W, H, 3, &image[0], 0);

	return 0;
}