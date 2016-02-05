#include <iostream>
#include <cstdlib>

#include <PartyKel/glm.hpp>
#include <PartyKel/WindowManager.hpp>

#include <PartyKel/renderer/FlagRenderer3D.hpp>
#include <PartyKel/renderer/TrackballCamera.hpp>

#include <vector>

static const Uint32 WINDOW_WIDTH = 1024;
static const Uint32 WINDOW_HEIGHT = 768;

using namespace PartyKel;

// Calcule une force de type ressort de Hook entre deux particules de positions P1 et P2
// K est la résistance du ressort et L sa longueur à vide
inline glm::vec3 hookForce(float K, float L, const glm::vec3& P1, const glm::vec3& P2) {
    static const float epsilon = 0.0001;

    glm::vec3 force = K * (1- (L/ std::max(glm::distance(P1,P2), epsilon) ) ) * (P2-P1);
    // force = glm::vec3(0);
    // std::cout << force.x << " " << force.y << " " << std::endl;
    return force;
}

// Calcule une force de type frein cinétique entre deux particules de vélocités v1 et v2
// V est le paramètre du frein et dt le pas temporel
inline glm::vec3 brakeForce(float V, float dt, const glm::vec3& v1, const glm::vec3& v2) {
    
        glm::vec3 force = V * ((v2-v1)/dt);
        return force;
}




// Structure permettant de simuler un drapeau à l'aide un système masse-ressort
struct Flag {
    int gridWidth, gridHeight; // Dimensions de la grille de points

    // Propriétés physique des points:
    std::vector<glm::vec3> positionArray;
    std::vector<glm::vec3> velocityArray;
    std::vector<float> massArray;
    std::vector<glm::vec3> forceArray;

    // Paramètres des forces interne de simulation
    // Longueurs à vide
    glm::vec2 L0;
    float L1;
    glm::vec2 L2;

    float K0, K1, K2; // Paramètres de résistance
    float V0, V1, V2; // Paramètres de frein

    // Créé un drapeau discretisé sous la forme d'une grille contenant gridWidth * gridHeight
    // points. Chaque point a pour masse mass / (gridWidth * gridHeight).
    // La taille du drapeau en 3D est spécifié par les paramètres width et height
    Flag(float mass, float width, float height, int gridWidth, int gridHeight):
        gridWidth(gridWidth), gridHeight(gridHeight),
        positionArray(gridWidth * gridHeight),
        velocityArray(gridWidth * gridHeight, glm::vec3(0.f)),
        // massArray(gridWidth * gridHeight, mass / (gridWidth * gridHeight)),
        massArray(gridWidth * gridHeight, 1),
        forceArray(gridWidth * gridHeight, glm::vec3(0.f)) {
        glm::vec3 origin(-0.5f * width, -0.5f * height, 0.f);
        glm::vec3 scale(width / (gridWidth - 1), height / (gridHeight - 1), 1.f);

        for(int j = 0; j < gridHeight; ++j) {
            for(int i = 0; i < gridWidth; ++i) {
                positionArray[i + j * gridWidth] = origin + glm::vec3(i, j, origin.z) * scale * 1.5f;
                massArray[i + j * gridWidth] = 1 - ( i / (2*(gridHeight*gridWidth)));
            }
        }

        // Les longueurs à vide sont calculés à partir de la position initiale
        // des points sur le drapeau
        L0.x = scale.x;
        L0.y = scale.y;
        L1 = glm::length(L0);
        L2 = 2.f * L0;

        // Ces paramètres sont à fixer pour avoir un système stable: HAVE FUN !
        K0 = 0.01;
        K1 = 1.f;
        K2 = 1.f;

        V0 = 0.1f;
        V1 = 0.1f;
        V2 = 0.1f;
    }


    // Applique les forces internes sur chaque point du drapeau SAUF les points fixes
    void applyInternalForces(float dt) {
        
        // On applique la Hook Force et le Frein Cinétique sur chaque couple de particule
         for(int j = 0; j < gridHeight-1; ++j) {
            for(int i = 1; i < gridWidth-1; ++i) {
                int k = i + j * gridWidth;
                forceArray[k] += hookForce(K0, L0.x, positionArray[k], positionArray[k + 1]) + hookForce(K0, L0.y, positionArray[k], positionArray[k + gridWidth]);
                forceArray[k+1] -= hookForce(K0, L0.x, positionArray[k], positionArray[k + 1]);
                forceArray[k + gridWidth] -= hookForce(K0, L0.y, positionArray[k], positionArray[k + gridWidth]);

                forceArray[k] += brakeForce(V0, dt, velocityArray[k], velocityArray[k+1]) + brakeForce(V0, dt, velocityArray[k], velocityArray[k+gridWidth]);
                forceArray[k+1] -= brakeForce(V0, dt, velocityArray[k], velocityArray[k+1]);
                forceArray[k+gridWidth] -= brakeForce(V0, dt, velocityArray[k], velocityArray[k+gridWidth]);

            }
        }
        // première colonne
        for(int j = 0; j < gridHeight; ++j) {
            int k = j;
            // positionArray[k] += hookForce(K0, L0.y, positionArray[k], positionArray[k + 1]);
            forceArray[k + 1] -= hookForce(K0, L0.y, positionArray[k], positionArray[k + 1]);

            forceArray[k + 1] -= brakeForce(V0, dt, velocityArray[k], velocityArray[k+1]);
        }
        // dernière colonne
        for(int j = 0; j < gridHeight-1; ++j) {
            int k = j * gridWidth;
            forceArray[k] += hookForce(K0, L0.y, positionArray[k], positionArray[k + gridWidth]);
            forceArray[k + gridWidth] -= hookForce(K0, L0.y, positionArray[k], positionArray[k + gridWidth]);

            forceArray[k] += brakeForce(V0, dt, velocityArray[k], velocityArray[k+gridWidth]);
            forceArray[k + gridWidth] -= brakeForce(V0, dt, velocityArray[k], velocityArray[k + gridWidth]);

        }
        // dernière colonne et dernière ligne (la particule en bas à droite)
        forceArray[gridHeight-2] += hookForce(K0, L0.y, positionArray[gridHeight-2], positionArray[gridHeight-1]);
        forceArray[gridHeight-1] -= hookForce(K0, L0.y, positionArray[gridHeight-2], positionArray[gridHeight-1]);

        forceArray[gridHeight-2] += brakeForce(V0, dt, velocityArray[gridHeight-2], velocityArray[gridHeight-1]);
        forceArray[gridHeight-1] -= brakeForce(V0, dt, velocityArray[gridHeight-2], velocityArray[gridHeight-1]);
       




    }

    // Applique une force externe sur chaque point du drapeau SAUF les points fixes
    void applyExternalForce(const glm::vec3& F) {
    
        for(int j = 0; j < gridHeight; ++j) {
            for(int i = 0; i < gridWidth; ++i) {
                int k = i + j * gridWidth;
                if(k%gridWidth!=0){
                    forceArray[k] += F;     
                }
            }
        }


    }

    void leapFrog(float dt){

        for(int j = 0; j < gridHeight; ++j) {
            for(int i = 0; i < gridWidth; ++i) {
                int k = i + j * gridWidth;
                velocityArray[k] += dt * forceArray[k]/massArray[k];
                positionArray[k] += dt * velocityArray[k];

            }
        }
        
    }

    // Met à jour la vitesse et la position de chaque point du drapeau
    // en utilisant un schema de type Leapfrog
    void update(float dt) {
        // TODO
        // Ne pas oublier de remettre les forces à 0 !

        leapFrog(dt);

        for(int s=0; s < forceArray.size(); ++s){
            forceArray[s] = glm::vec3(0.f);
        }

    }


};

int main() {
    WindowManager wm(WINDOW_WIDTH, WINDOW_HEIGHT, "Fun with Flags");
    wm.setFramerate(30);


    Flag flag(4096.f, 2, 1.5, 32, 16); // Création d'un drapeau // Flag(float mass, float width, float height, int gridWidth, int gridHeight)
    glm::vec3 G(0.f, -0.001f, 0.f); // Gravité

    FlagRenderer3D renderer(flag.gridWidth, flag.gridHeight);
    renderer.setProjMatrix(glm::perspective(70.f, float(WINDOW_WIDTH) / WINDOW_HEIGHT, 0.1f, 100.f));

    TrackballCamera camera;
    int mouseLastX, mouseLastY;

    // Temps s'écoulant entre chaque frame
    float dt = 0.f;

	bool done = false;
    bool wireframe = true;
    while(!done) {
        wm.startMainLoop();

        // Rendu
        renderer.clear();

        renderer.setViewMatrix(camera.getViewMatrix());
        renderer.drawGrid(flag.positionArray.data(), wireframe);

        // Simulation
        if(dt > 0.f) {
            flag.applyExternalForce(G); // Applique la gravité
            flag.applyExternalForce(glm::sphericalRand(0.1f)); // Applique un "vent" de direction aléatoire et de force 0.1 Newtons
            flag.applyInternalForces(dt); // Applique les forces internes
            flag.update(dt); // Mise à jour du système à partir des forces appliquées
        }

        // Gestion des evenements
		SDL_Event e;
        while(wm.pollEvent(e)) {
			switch(e.type) {
				default:
					break;
				case SDL_QUIT:
					done = true;
					break;
                case SDL_KEYDOWN:
                    if(e.key.keysym.sym == SDLK_SPACE) {
                        wireframe = !wireframe;
                    }
                case SDL_MOUSEBUTTONDOWN:
                    if(e.button.button == SDL_BUTTON_WHEELUP) {
                        camera.moveFront(0.2f);
                    } else if(e.button.button == SDL_BUTTON_WHEELDOWN) {
                        camera.moveFront(-0.2f);
                    } else if(e.button.button == SDL_BUTTON_LEFT) {
                        mouseLastX = e.button.x;
                        mouseLastY = e.button.y;
                    }
			}
		}

        int mouseX, mouseY;
        if(SDL_GetMouseState(&mouseX, &mouseY) & SDL_BUTTON(SDL_BUTTON_LEFT)) {
            float dX = mouseX - mouseLastX, dY = mouseY - mouseLastY;
            camera.rotateLeft(glm::radians(dX));
            camera.rotateUp(glm::radians(dY));
            mouseLastX = mouseX;
            mouseLastY = mouseY;
        }

        // Mise à jour de la fenêtre
        dt = wm.update();
	}

	return EXIT_SUCCESS;
}
