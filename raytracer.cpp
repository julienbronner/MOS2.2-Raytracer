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
#include <sstream>

#include <random>
static std::default_random_engine engine(10); //random seed 10
static std::uniform_real_distribution<double> uniform(0, 1);

class Vector { //Classe pour définir un vecteur, défini egalement un point et une couleur, donc défini une entité à trois coordonnées
public:
	explicit Vector(double x = 0, double y = 0, double z = 0) {
		coords[0] = x;
		coords[1] = y;
		coords[2] = z;
	};
	double operator[](int i) const { return coords[i]; };
	double& operator[](int i) { return coords[i]; };
	Vector& operator+=(const Vector& a) {
		coords[0] += a[0];
		coords[1] += a[1];
		coords[2] += a[2];
		return *this;
	};
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
Vector operator-(const Vector& b) {
	return Vector( -b[0], -b[1], -b[2]);
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
Vector cross(const Vector& a, const Vector& b) {
	return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}
double sqr(const double& a) {
	return a * a;
}
double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}


Vector random_cos(const Vector& N) {
	double u1 = uniform(engine);
	double u2 = uniform(engine);
	double x = sqrt(1 - u1) * cos(2 * M_PI * u2);
	double y = sqrt(1 - u1) * sin(2 * M_PI * u2);
	double z = sqrt(u1);
	Vector T1;
	if (N[0] < N[1] && N[0] < N[2]) {
		T1 = Vector(0, -N[2], N[1]);
	}
	else if (N[1] < N[0] && N[1] < N[2]) {
		T1 = Vector(N[2], 0, -N[0]);
	}
	else {
		T1 = Vector(-N[1], N[0], 0);
	}
	Vector T2 = cross(N, T1);
	//return z * N + x * T1 + y * T2;
	return z * N - x * T1 - y * T2;

}

class Ray { // un rayon qui a donc une direction C et un point d'origine u
public:
	Ray(const Vector& C, const Vector& u) : C(C), u(u) {
	}
	Vector C;
	Vector u;
};

class Sphere { // Définition d'une sphere
	/* Origine O, rayon R, couleur/albedo rho
	* indice de refraction n_refraction
	* Booléen qui disent si la sphere est un mirroir, si elle est transparente, 
	* ou si c'est une sphere inversée pour pouvoir définir une sphere creuse
	*/
public:
	Sphere(const Vector& O, double R, Vector rho, double n_refraction, bool mirror, bool transparency, bool sphere_inverse) : O(O), R(R), rho(rho), n_refraction(n_refraction), mirror(mirror), transparency(transparency), sphere_inverse(sphere_inverse){
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
			if (sphere_inverse) {
				N = -N;
			}
			return true;
		}
	}
	Vector O;
	double R, n_refraction;
	Vector rho;
	bool mirror, transparency, sphere_inverse;
};

class Light { // Lumière qui à une puissance I et une position L
public:
	Light(const double& I, const Vector& L) : I(I), L(L) {
	}
	double I;
	Vector L;
};

class Scene { // Scene globale, dans laquelle on insert nos objets
	/* elle est définie par une lumière
	une couleur de fond (noir généralement)
	et un indice de refraction, 1 comme l'air de base
	*/
public:
	Scene(Light& Lum, Vector& color, double n_refrac_scene) : Lum(Lum), color(color), n_refrac_scene(n_refrac_scene) {
	}
	bool intersect_scene(const Ray& rayon, Vector& P, Vector& N, Vector& albedo, double& t, double& n_refraction, bool& mirror, bool& transparency) {
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
				n_refraction = S.n_refraction;
				mirror = S.mirror;
				transparency = S.transparency;
				bool_sortie = true;
				//Vector PL;
				//PL = Lum.L - P;
				//couleur = Lum.I / (4 * M_PI * PL.sqrNorm()) * std::max(0., dot(N, PL.get_normalized())) * S.rho / M_PI;
			}
		}
		return bool_sortie;
	}
	Vector getColor(Ray& rayon, const int& rebond) {

		if (rebond > 10) return Vector(0., 0., 0.);

		Vector coul = color;
		Vector P, N, albedo;
		double t = 2147483647;
		double n_refraction;
		bool mirror, transparency;
		bool inter = intersect_scene(rayon, P, N, albedo, t, n_refraction, mirror, transparency);
		if (inter) {
			Vector PL;
			PL = Lum.L - P;
			double epsilon = 0.001;
			if (mirror) {
				// cas d'une sphere miroir
				Vector reflex = rayon.u - 2 * dot(rayon.u, N) * N;
				Ray rayon_reflechi(P + epsilon * reflex, reflex.get_normalized());
				return getColor( rayon_reflechi, rebond + 1);
			}
			else {
				if (transparency) {
					// cas d'une sphere transparente
					double n1 = n_refrac_scene, n2 = n_refraction;
					Vector N2 = N;
					if (dot(rayon.u, N) > 0) {
						std::swap(n1, n2);
						N2 = -N;
					}
					Vector t_t = n1 / n2 * (rayon.u - dot(rayon.u, N2) * N2);
					double radical = 1 - sqr(n1 / n2) * (1 - sqr(dot(rayon.u, N2)) );
					if (radical < 0) {
						Vector reflex = rayon.u - 2 * dot(rayon.u, N2) * N2;
						Ray rayon_reflechi(P + epsilon * N2, reflex.get_normalized());
						return getColor(rayon_reflechi, rebond + 1);
					}
					Vector t_n = -sqrt(radical) * N2;
					Vector vect_dir_refracte = t_t + t_n;
					Ray rayon_refracte(P - epsilon * N2, vect_dir_refracte);
					return getColor(rayon_refracte, rebond + 1);

				}
				else {
					// eclairage direct
					Vector shadowP, shadowN, shadowAlbedo;
					double shadowt = 2147483647;
					double shadow_nRefraction;
					bool shadowMirror, shadowTransparency;
					Ray shadowRay(P + epsilon * PL.get_normalized(), PL.get_normalized());
					bool shadowInter = intersect_scene(shadowRay, shadowP, shadowN, shadowAlbedo, shadowt, shadow_nRefraction, shadowMirror, shadowTransparency);
					if (shadowInter && shadowt < sqrt(PL.sqrNorm())) {
						coul = Vector(0., 0., 0.);
					}
					else {
						coul = Lum.I / (4 * M_PI * PL.sqrNorm()) * std::max(0., dot(N, PL.get_normalized())) * albedo / M_PI;
					}

					// eclairage indirect
					Vector random_vector = random_cos(N);
					Ray rayon_indirect(P + epsilon * random_vector, random_vector.get_normalized());
					coul += albedo * getColor(rayon_indirect, rebond + 1);
				}
			}
		}
		return coul;
	}

	std::vector<Sphere> objects;
	Light Lum;
	const Vector color;
	double n_refrac_scene;
};

void integrateCos() {
	double N = std::pow(10, 6);
	double sigma = 0.3;
	double s = 0;
	for (int i = 0; i < N; i++) {
		double u1 = uniform(engine);
		double u2 = uniform(engine);
		double xi = sigma * cos(2 * M_PI * u1) * sqrt(-2 * log(u2));
		if ((xi > -M_PI/2) || (xi < M_PI/2)){
			double p = 1 / (sigma * sqrt(2 * M_PI)) * exp(-xi * xi / (2 * sigma * sigma));
			s += std::pow(cos(xi), 10) / p;
		}
	}
	s = s / N;
	double s_exact = 63 * M_PI / 256;
	std::cout << "Résultat de Monte-Carlo " << s << std::endl;
	std::cout << "Résultat exact " << s_exact << std::endl;
}

void integrateCosDimQuatre() {
	double N = std::pow(10,6);
	double sigma = 1;
	double s = 0;
	for (int i = 0; i < N; i++) {
		double u1 = uniform(engine);
		double u2 = uniform(engine);
		double u3 = uniform(engine);
		double u4 = uniform(engine);
		double x_x = sigma * cos(2 * M_PI * u1) * sqrt(-2 * log(u2));
		double x_y = sigma * sin(2 * M_PI * u1) * sqrt(-2 * log(u2));
		double x_z = sigma * cos(2 * M_PI * u3) * sqrt(-2 * log(u4));
		double x_w = sigma * sin(2 * M_PI * u3) * sqrt(-2 * log(u4));

		if (((x_x > -M_PI / 2) && (x_x < M_PI / 2)) && ((x_y > -M_PI / 2) && (x_y < M_PI / 2)) && ((x_z > -M_PI / 2) && (x_z < M_PI / 2)) && ((x_w > -M_PI / 2) && (x_w < M_PI / 2))) {
			double p_x = 1 / (sigma * sqrt(2 * M_PI)) * exp(-x_x * x_x / (2 * sigma * sigma));
			double p_y = 1 / (sigma * sqrt(2 * M_PI)) * exp(-x_y * x_y / (2 * sigma * sigma));
			double p_z = 1 / (sigma * sqrt(2 * M_PI)) * exp(-x_z * x_z / (2 * sigma * sigma));
			double p_w = 1 / (sigma * sqrt(2 * M_PI)) * exp(-x_w * x_w / (2 * sigma * sigma));
			s += std::pow(cos(x_x + x_y + x_z + x_w), 2) / (p_x * p_y * p_z * p_w);
		}
	}
	s = s / N;
	std::cout << "Resultat de Monte-Carlo " << s << std::endl;
}


int main() {
	
	auto start = std::chrono::high_resolution_clock::now();
	int W = 512;
	int H = 512;

	Vector C(0, 0, 55);
	int r = 10;
	Sphere S0(Vector(0, 0, 0), r, Vector(1, 1, 1), 1.4, false, false, false);
	Sphere S1(Vector(-20, 0, 0), r, Vector(1, 0, 0), 1.4, true, false, false);
	Sphere S2(Vector(0, 0, 0), r, Vector(1, 0, 0), 1.4, false, true, false);
	Sphere S3(Vector(20, 0, 0), r, Vector(0, 0, 1), 1.4, false, true, false);
	Sphere S4(Vector(20, -0, 0), 9.5, Vector(0, 1, 1), 1.4, false, true, true);
	Sphere SMurFace(Vector(0, 0, -1000), 940, Vector(0, 1, 0), 1.4, false, false, false);
	Sphere SMurDos(Vector(0, 0, 1000), 940, Vector(1, 0, 1), 1.4, false, false, false);
	Sphere SMurHaut(Vector(0, 1000, 0), 940, Vector(1, 0, 0), 1.4, false, false, false);
	Sphere SMurBas(Vector(0, -1000, 0), 990, Vector(0, 0, 1), 1.4, false, false, false);
	Sphere SMurDroite(Vector(1000, 0, 0), 940, Vector(0, 1, 1), 1.4, false, false, false);
	Sphere SMurGauche(Vector(-1000, 0, 0), 940, Vector(1, 1, 0), 1.4, false, false, false);

	double fov = 60 * M_PI / 180;
	Light Lum(double(4E9), Vector(-10, 20, 40));
	Vector couleur(0, 0, 0);
	Scene scene(Lum, couleur, 1.);
	scene.objects.push_back(S0);
	//scene.objects.push_back(S1);
	//scene.objects.push_back(S2);
	//scene.objects.push_back(S3);
	//scene.objects.push_back(S4);
	double gamma = 0.45;
	
	scene.objects.push_back(SMurFace);
	scene.objects.push_back(SMurDos);
	scene.objects.push_back(SMurHaut);
	scene.objects.push_back(SMurBas);
	scene.objects.push_back(SMurDroite);
	scene.objects.push_back(SMurGauche);

	int nb_ray = 100;

	std::vector<unsigned char> image(W * H * 3, 0);
	#pragma omp parallel for schedule(dynamic,1)
	for (int i = 0; i < H; i++) {

		for (int j = 0; j < W; j++) {

			Vector coul = scene.color;


			//Vector coul = scene.getColor(rayon, int(0));
			//Vector coul = (scene.getColor(rayon, int(0)) + scene.getColor(rayon, int(0)))/2;

			for(int k=0; k<nb_ray; k++){
				double u1 = uniform(engine);
				double u2 = uniform(engine);
				double x = sqrt(1 - u1) * cos(2 * M_PI * u2);
				double y = sqrt(1 - u1) * sin(2 * M_PI * u2);

				Vector u(j - W / 2 + x, i - H / 2 + y, -W / (2 * tan(fov / 2)));
				u = u.get_normalized();
				Ray rayon(C, u);
				coul += scene.getColor(rayon, int(0));
			}
			coul = coul / nb_ray;
			
			image[((H - i - 1) * W + j) * 3 + 0] = std::min(255., std::pow(coul[0], gamma) );
			image[((H - i - 1) * W + j) * 3 + 1] = std::min(255., std::pow(coul[1], gamma) );
			image[((H - i - 1) * W + j) * 3 + 2] = std::min(255., std::pow(coul[2], gamma) );

		}
	}
	auto end = std::chrono::high_resolution_clock::now();
	auto diff = end - start;
	auto diff_sec = std::chrono::duration_cast<std::chrono::milliseconds>(diff);
	std::ostringstream out;
	std::string Path = "D:/julbr/Documents/ecole/ECL/3A/MOS_2.2_Informatique_Graphique/Image/";
	out << Path << "new_test-" << nb_ray <<"_rayons-" << diff_sec.count() << "ms.png";
	std::string location = out.str();
	char* cname;
	cname = &location[0];
	stbi_write_png(cname, W, H, 3, &image[0], 0);
	std::cout << "La calcul a dure : " << diff_sec.count() << " millisecondes" << std::endl;
	//stbi_write_png("azertyuiop2555.png", W, H, 3, &image[0], 0);

	return 0;
}