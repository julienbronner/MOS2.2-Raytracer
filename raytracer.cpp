#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "Template_creation_image/stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "Template_creation_image/stb_image.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>

#include <list>

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
Vector operator*(const Vector& b, double a) {
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
	T1 = T1.get_normalized();
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

class Object {
public:
	Object() {}
	virtual bool intersect(const Ray& r, Vector& P, Vector& normale, double& t) = 0;
	Vector rho;
	double n_refraction;
	bool mirror, transparency;
};

class Sphere : public Object { // Définition d'une sphere
	/* Origine O, rayon R, couleur/albedo rho
	* indice de refraction n_refraction
	* Booléen qui disent si la sphere est un mirroir, si elle est transparente, 
	* ou si c'est une sphere inversée pour pouvoir définir une sphere creuse
	*/
public:
	Sphere(const Vector& O, double R, Vector rho, double n_refraction, bool mirror, bool transparency, bool sphere_inverse) : O(O), R(R), sphere_inverse(sphere_inverse){
		this->rho = rho;
		this->n_refraction = n_refraction;
		this->mirror = mirror;
		this->transparency = transparency;
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
	double R;
	bool sphere_inverse;
};

class Light { // Lumière qui à une puissance I et une position L
public:
	Light(const double& I, const Vector& L) : I(I), L(L) {
	}
	double I;
	Vector L;
};

class BoundingBox {
public:
	bool intersect(const Ray& r) {
		double t1x = (mini[0] - r.C[0]) / r.u[0];
		double t2x = (maxi[0] - r.C[0]) / r.u[0];
		double txMin = std::min(t1x, t2x);
		double txMax = std::max(t1x, t2x);

		double t1y = (mini[1] - r.C[1]) / r.u[1];
		double t2y = (maxi[1] - r.C[1]) / r.u[1];
		double tyMin = std::min(t1y, t2y);
		double tyMax = std::max(t1y, t2y);

		double t1z = (mini[2] - r.C[2]) / r.u[2];
		double t2z = (maxi[2] - r.C[2]) / r.u[2];
		double tzMin = std::min(t1z, t2z);
		double tzMax = std::max(t1z, t2z);

		double tMax = std::min(txMax, std::min(tyMax, tzMax));
		double tMin = std::max(txMin, std::max(tyMin, tzMin));

		if (tMax < 0) {
			return false;
		};
		return tMax > tMin;
	}
	Vector mini, maxi;
};

class Noeud {
public:
	Noeud* fg, * fd;
	BoundingBox b;
	int debut, fin_exclu;
};

// Code simple obj file reader
#include <string>
#include <stdio.h>
class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
	};
	int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
	int uvi, uvj, uvk;  // indices within the uv coordinates array
	int ni, nj, nk;  // indices within the normals array
	int group;       // face group
};


class TriangleMesh : public Object {
public:
	~TriangleMesh() {}
	TriangleMesh(const Vector& albedo, double n_refraction = 1., bool mirror = false, bool transparency = false) {
		this->rho = albedo;
		this->n_refraction = n_refraction;
		this->mirror = mirror;
		this->transparency = transparency;
	};

	BoundingBox buildBB(int debut, int fin) { //debut et fin sont des indices de triangle
		BoundingBox BB;
		BB.mini = Vector(1E9, 1E9, 1E9);
		BB.maxi = Vector(-1E9, -1E9, -1E9);
		for (int i = debut; i < fin; i++) {
			for(int j = 0; j< 3; j++){
				BB.mini[j] = std::min(BB.mini[j], vertices[indices[i].vtxi][j]);
				BB.mini[j] = std::min(BB.mini[j], vertices[indices[i].vtxj][j]);
				BB.mini[j] = std::min(BB.mini[j], vertices[indices[i].vtxk][j]);

				BB.maxi[j] = std::max(BB.maxi[j], vertices[indices[i].vtxi][j]);
				BB.maxi[j] = std::max(BB.maxi[j], vertices[indices[i].vtxj][j]);
				BB.maxi[j] = std::max(BB.maxi[j], vertices[indices[i].vtxk][j]);
			}
		}
		return BB;
	};

	void buildBVH(Noeud* n, int debut, int fin_exclu) {
		n->debut = debut;
		n->fin_exclu = fin_exclu;
		n->b = buildBB(debut, fin_exclu);
		Vector diag = n->b.maxi - n->b.mini;
		int dim;
		if (diag[0] >= diag[1] && diag[0] >= diag[2]) {
			dim = 0;
		}
		else {
			if (diag[1] >= diag[0] && diag[1] >= diag[2]) {
				dim = 1;
			}
			else {
				dim = 2;
			}
		}
		double milieu = (n->b.mini[dim] + n->b.maxi[dim]) /2.;
		int indice_pivot = n->debut;
		for (int i = n->debut; i < n->fin_exclu; i++) {
			double milieu_triangle = (vertices[indices[i].vtxi][dim] + vertices[indices[i].vtxj][dim] + vertices[indices[i].vtxk][dim]) / 3.;
			if (milieu_triangle < milieu) {
				std::swap(indices[i], indices[indice_pivot]);
				indice_pivot++;
			}
		}

		n->fg = NULL;
		n->fd = NULL;
		int nbr_triangle_critere_arret = 5;
		if (indice_pivot == debut || indice_pivot == fin_exclu || (fin_exclu - debut < nbr_triangle_critere_arret)) {
			return;
		}

		n->fg = new Noeud;
		n->fd = new Noeud;

		buildBVH(n->fg, n->debut, indice_pivot);
		buildBVH(n->fd, indice_pivot, n->fin_exclu);

	}

	void readOBJ(const char* obj) {

		char matfile[255];
		char grp[255];

		FILE* f;
		f = fopen(obj, "r");
		int curGroup = -1;
		while (!feof(f)) {
			char line[255];
			if (!fgets(line, 255, f)) break;

			std::string linetrim(line);
			linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
			strcpy(line, linetrim.c_str());

			if (line[0] == 'u' && line[1] == 's') {
				sscanf(line, "usemtl %[^\n]\n", grp);
				curGroup++;
			}

			if (line[0] == 'v' && line[1] == ' ') {
				Vector vec;

				Vector col;
				if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
					col[0] = std::min(1., std::max(0., col[0]));
					col[1] = std::min(1., std::max(0., col[1]));
					col[2] = std::min(1., std::max(0., col[2]));

					vertices.push_back(vec);
					vertexcolors.push_back(col);

				}
				else {
					sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
					vertices.push_back(vec);
				}
			}
			if (line[0] == 'v' && line[1] == 'n') {
				Vector vec;
				sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
				normals.push_back(vec);
			}
			if (line[0] == 'v' && line[1] == 't') {
				Vector vec;
				sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
				uvs.push_back(vec);
			}
			if (line[0] == 'f') {
				TriangleIndices t;
				int i0, i1, i2, i3;
				int j0, j1, j2, j3;
				int k0, k1, k2, k3;
				int nn;
				t.group = curGroup;

				char* consumedline = line + 1;
				int offset;

				nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
				if (nn == 9) {
					if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
					if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
					if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
					if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
					if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
					if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
					if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
					if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
					if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
					indices.push_back(t);
				}
				else {
					nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
					if (nn == 6) {
						if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
						if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
						if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
						if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
						if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
						if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
						indices.push_back(t);
					}
					else {
						nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
						if (nn == 3) {
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							indices.push_back(t);
						}
						else {
							nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
							if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
							if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
							indices.push_back(t);
						}
					}
				}

				consumedline = consumedline + offset;

				while (true) {
					if (consumedline[0] == '\n') break;
					if (consumedline[0] == '\0') break;
					nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
					TriangleIndices t2;
					t2.group = curGroup;
					if (nn == 3) {
						if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
						if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
						if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
						if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
						if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
						if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
						if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
						if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
						if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
						indices.push_back(t2);
						consumedline = consumedline + offset;
						i2 = i3;
						j2 = j3;
						k2 = k3;
					}
					else {
						nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
						if (nn == 2) {
							if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
							if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
							if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
							if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
							if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
							if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
							consumedline = consumedline + offset;
							i2 = i3;
							j2 = j3;
							indices.push_back(t2);
						}
						else {
							nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
							if (nn == 2) {
								if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
								if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
								if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
								if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
								if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
								if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
								consumedline = consumedline + offset;
								i2 = i3;
								k2 = k3;
								indices.push_back(t2);
							}
							else {
								nn = sscanf(consumedline, "%u%n", &i3, &offset);
								if (nn == 1) {
									if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
									if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
									if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
									consumedline = consumedline + offset;
									i2 = i3;
									indices.push_back(t2);
								}
								else {
									consumedline = consumedline + 1;
								}
							}
						}
					}
				}

			}

		}
		fclose(f);

	}

	bool intersect(const Ray& r, Vector& P, Vector& normale, double& t) {
		if (!BVH->b.intersect(r)) return false;

		t = 1E10;
		bool intersect_bool = false;

		std::list<Noeud*> l;
		l.push_back(BVH);
		while (!l.empty()) {
			Noeud* courant = l.front();
			l.pop_front();

			if (courant->fg) {
				if (courant->fg->b.intersect(r)) {
					l.push_front(courant->fg);
				}
				if (courant->fd->b.intersect(r)) {
					l.push_front(courant->fd);
				}
			}
			else {
				for (int i = courant->debut; i < courant->fin_exclu; i++) {
					const Vector& A = vertices[indices[i].vtxi];
					const Vector& B = vertices[indices[i].vtxj];
					const Vector& C = vertices[indices[i].vtxk];

					Vector e1 = B - A;
					Vector e2 = C - A;
					Vector N = cross(e1, e2);
					Vector AO = r.C - A;
					Vector AOu = cross(AO, r.u);
					double invUN = 1. / dot(r.u, N);
					double beta = -dot(e2, AOu) * invUN;
					double gamma = dot(e1, AOu) * invUN;
					double alpha = 1 - beta - gamma;
					double localt = -dot(AO, N) * invUN;
					if (beta >= 0 && gamma >= 0 && beta <= 1 && gamma <= 1 && alpha >= 0 && localt >= 0) {
						intersect_bool = true;
						if (localt < t) {
							t = localt;
							normale = N.get_normalized();
							P = r.C + t * r.u;
						}
					}
				}
			}
		}

		return intersect_bool;
	}

	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;
	Noeud* BVH = new Noeud;
};
// fin code simple obj file reader


class Scene { // Scene globale, dans laquelle on insert nos objets
	/* elle est définie par une lumière
	une couleur de fond (noir généralement)
	et un indice de refraction, 1 comme l'air de base
	*/
public:
	Scene(Light& Lum, Vector& color, double n_refrac_scene) : Lum(Lum), color(color), n_refrac_scene(n_refrac_scene) {
	}
	bool intersect_scene(const Ray& rayon, Vector& P, Vector& N, double& t, int& objectId) {
		bool bool_sortie = false;
		for (int i = 0; i < objects.size(); i++) {
			Vector P_current, N_current;
			double t_current;
			bool inter = objects[i]->intersect(rayon, P_current, N_current, t_current);
			if (inter && (t_current < t)) {
				P = P_current;
				N = N_current;
				t = t_current;
				//sphere_intersect = objects[i];
				/*albedo = objects[i].rho;
				n_refraction = objects[i].n_refraction;
				mirror = objects[i].mirror;
				transparency = objects[i].transparency;
				R_sphere = objects[i].R;*/
				objectId = i;
				bool_sortie = true;
				//Vector PL;
				//PL = Lum.L - P;
				//couleur = Lum.I / (4 * M_PI * PL.sqrNorm()) * std::max(0., dot(N, PL.get_normalized())) * S.rho / M_PI;
			}
		}
		return bool_sortie;
	}

	Vector getColor(Ray& rayon, const int& rebond, bool lastDiffuse) {

		if (rebond > 10) return Vector(0., 0., 0.);

		Vector coul = color;
		Vector P, N;
		double t = 2147483647;
		/*double n_refraction, R_sphere;
		bool mirror, transparency;*/
		int objectId;
		
		//Sphere sphere_intersect(Vector(0,0,0), 0., Vector(0,0,0), 1, false, false, false);
		bool inter = intersect_scene(rayon, P, N, t, objectId);
		if (inter) {

			if (objectId == 0) {
				if (rebond == 0 || !lastDiffuse) {
					double sqr_R = sqr(dynamic_cast<Sphere*>(objects[0])->R);
					return Vector(Lum.I, Lum.I, Lum.I) / (4 * M_PI * M_PI * sqr_R);
				}

				else {
					return Vector(0., 0., 0.);
				}
			}
			else {
				Vector PL;
				PL = Lum.L - P;
				double epsilon = 0.001;
				if (objects[objectId]->mirror) {
					// cas d'une sphere miroir
					Vector reflex = rayon.u - 2 * dot(rayon.u, N) * N;
					Ray rayon_reflechi(P + epsilon * reflex, reflex.get_normalized());
					
					return getColor(rayon_reflechi, rebond + 1, false);
				}
				else {
					if (objects[objectId]->transparency) {
						// cas d'une sphere transparente
						double n1 = n_refrac_scene, n2 = objects[objectId]->n_refraction;
						Vector N2 = N;
				
						if (dot(rayon.u, N) > 0) {
							std::swap(n1, n2);
							N2 = -N;
						}
						Vector t_t = n1 / n2 * (rayon.u - dot(rayon.u, N2) * N2);
						double radical = 1 - sqr(n1 / n2) * (1 - sqr(dot(rayon.u, N2)));
						if (radical < 0) {
							Vector reflex = rayon.u - 2 * dot(rayon.u, N2) * N2;
							Ray rayon_reflechi(P + epsilon * N2, reflex.get_normalized());
							return getColor(rayon_reflechi, rebond + 1, false);
						}
						Vector t_n = -sqrt(radical) * N2;
						Vector vect_dir_refracte = t_t + t_n;
						Ray rayon_refracte(P - epsilon * N2, vect_dir_refracte);
						return getColor(rayon_refracte, rebond + 1, false);

					}
					else {
						PL = PL.get_normalized();
						Vector w = random_cos(-PL); // on veut LP
						Vector xprime = w * dynamic_cast<Sphere*>(objects[0])->R + dynamic_cast<Sphere*>(objects[0])->O;
						Vector Pxprime = xprime - P;
						double d = sqrt(Pxprime.sqrNorm()); 
						Pxprime = Pxprime / d;

						Vector shadowP, shadowN, shadowAlbedo;
						int shadow_objectId;
						double shadowt = 2147483647;
						//double shadow_nRefraction, shadow_R_Sphere;
						//bool shadowMirror, shadowTransparency;
						Ray shadowRay(P + epsilon * N, Pxprime);
						//Sphere sphere_intersect_shadow(Vector(0, 0, 0), 0., Vector(0, 0, 0), 1, false, false, false);
						bool shadowInter = intersect_scene(shadowRay, shadowP, shadowN, shadowt, shadow_objectId);
						if (shadowInter && shadowt < d-epsilon*10) {
							coul = Vector(0., 0., 0.);
						}
						else {
							double sqr_R = sqr(dynamic_cast<Sphere*>(objects[0])->R);
							double proba = std::max(0. ,dot(-PL, w)) / (M_PI* sqr_R);
							double J = std::max(0., dot(w, -Pxprime))/(d*d);
							coul = Lum.I / (4 * M_PI * sqr_R) * std::max(0., dot(N, Pxprime)) * objects[objectId]->rho / M_PI * J / proba;
						}

						// eclairage indirect
						Vector random_vector = random_cos(N);
						Ray rayon_indirect(P + epsilon * random_vector, random_vector.get_normalized());
						coul += objects[objectId]->rho * getColor(rayon_indirect, rebond + 1, true);
					}
				}
			}
		}
		return coul;
	}

	std::vector<Object*> objects;
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

	Light Lum(double(4E9), Vector(-10, 20, 40));
	Vector couleur(0, 0, 0);
	Scene scene(Lum, couleur, 1.);

	Vector C(0, 0, 55);
	int r = 10;

	Sphere SLum(scene.Lum.L, 5, Vector(1, 1, 1), 1.4, false, false, false);
	Sphere S0(Vector(20, 0, 15), r, Vector(1, 1, 1), 1.4, false, false, false);
	Sphere S1(Vector(0, 0, 0), r, Vector(1, 1, 1), 1.4, false, false, false);
	Sphere S2(Vector(-20, 0, -15), r, Vector(1, 1, 1), 1.4, false, false, false);
	Sphere S_miroir_gauche(Vector(-20, 0, 0), r, Vector(1, 0, 0), 1.4, true, false, false);
	Sphere S_transparente_centre(Vector(0, 0, 0), r, Vector(1, 0, 0), 1.4, false, true, false);
	Sphere S_creuse_exterieur_droite(Vector(20, 0, 0), r, Vector(0, 0, 1), 1.4, false, true, false);
	Sphere S_creuse_interieur_droite(Vector(20, -0, 0), 9.5, Vector(0, 1, 1), 1.4, false, true, true);
	Sphere SMurFace(Vector(0, 0, -1000), 940, Vector(0, 0.5, 0), 1.4, false, false, false);
	Sphere SMurDos(Vector(0, 0, 1000), 940, Vector(0.5, 0, 0.5), 1.4, false, false, false);
	Sphere SMurHaut(Vector(0, 1000, 0), 940, Vector(0.5, 0, 0), 1.4, false, false, false);
	Sphere SMurBas(Vector(0, -1000, 0), 990, Vector(0, 0, 0.5), 1.4, false, false, false);
	Sphere SMurDroite(Vector(1000, 0, 0), 940, Vector(0, 0.5, 0.5), 1.4, false, false, false);
	Sphere SMurGauche(Vector(-1000, 0, 0), 940, Vector(0.5, 0.5, 0), 1.4, false, false, false);

	TriangleMesh m(Vector(1., 1., 1.));
	m.readOBJ("D:/julbr/Documents/ecole/ECL/3A/MOS_2.2_Informatique_Graphique/Maillages/Australian_Cattle_Dog_v1_L3.123c9c6a5764-399b-4e86-9897-6bcb08b5e8ed/13463_Australian_Cattle_Dog_v3.obj");
	for (int i = 0; i < m.vertices.size(); i++) {
		std::swap(m.vertices[i][1], m.vertices[i][2]);
		m.vertices[i][2] = -m.vertices[i][2];
		m.vertices[i][1] -= 10;
	}
	//m.buildBB();
	m.buildBVH(m.BVH, 0, m.indices.size());

	double fov = 60 * M_PI / 180;
	scene.objects.push_back(&SLum);
	//scene.objects.push_back(&S0);
	//scene.objects.push_back(&S1);
	//scene.objects.push_back(&S2);
	scene.objects.push_back(&m);
	//scene.objects.push_back(&S_miroir_gauche);
	//scene.objects.push_back(&S_transparente_centre);
	//scene.objects.push_back(&S_creuse_exterieur_droite);
	//scene.objects.push_back(&S_creuse_interieur_droite);
	double gamma = 0.45;
	
	scene.objects.push_back(&SMurFace);
	scene.objects.push_back(&SMurDos);
	scene.objects.push_back(&SMurHaut);
	scene.objects.push_back(&SMurBas);
	scene.objects.push_back(&SMurDroite);
	scene.objects.push_back(&SMurGauche);

	int nb_ray = 100;
	double distance_plan_nettete = 55.;
	double rayon_obturateur = 0.01;

	std::vector<unsigned char> image(W * H * 3, 0);
	#pragma omp parallel for schedule(dynamic,1)
	for (int i = 0; i < H; i++) {
		std::cout << "Pixel hauteur: " << i << std::endl;
		for (int j = 0; j < W; j++) {
			
			Vector coul = scene.color;

			//Vector coul = scene.getColor(rayon, int(0));
			//Vector coul = (scene.getColor(rayon, int(0)) + scene.getColor(rayon, int(0)))/2;

			for(int k=0; k<nb_ray; k++){
				double u1 = uniform(engine);
				double u2 = uniform(engine);
				double x = sqrt(1 - u1) * cos(2 * M_PI * u2);
				double y = sqrt(1 - u1) * sin(2 * M_PI * u2);

				double u3 = uniform(engine);
				double u4 = uniform(engine);
				double x_capteur = u3 * 2 * rayon_obturateur - rayon_obturateur; // pour avoir des valeurs entre -rayon_obt et +rayon_obt
				double y_capteur = u4 * 2 * rayon_obturateur - rayon_obturateur;
				while( (x_capteur * x_capteur + y_capteur * y_capteur) > rayon_obturateur){ // permet d'avoir des valeurs dans le cercle de rayon obturateur
					u3 = uniform(engine);
					u4 = uniform(engine);
					x_capteur = u3 * 2 * rayon_obturateur - rayon_obturateur; // pour avoir des valeurs entre -rayon_obt et +rayon_obt
					y_capteur = u4 * 2 * rayon_obturateur - rayon_obturateur;
				}
				/*double x_capteur = sqrt(1 - u3) * cos(2 * M_PI * u4);
				double y_capteur = sqrt(1 - u3) * sin(2 * M_PI * u4);*/

				Vector u(j - W / 2 + x, i - H / 2 + y, -W / (2 * tan(fov / 2)));
				u = u.get_normalized();

				Vector cible = C + distance_plan_nettete * u;
				Vector Cprime = C + Vector(x_capteur, y_capteur, 0);
				Vector uprime = (cible - Cprime).get_normalized();

				Ray rayon(Cprime, uprime);
				bool lastDiffuse = false;
				coul += scene.getColor(rayon, int(0), lastDiffuse);
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