#include <iostream>
#include <cstdlib>

#include <PartyKel/glm.hpp>
#include <PartyKel/WindowManager.hpp>

#include <PartyKel/renderer/FlagRenderer3D.hpp>
#include <PartyKel/renderer/TrackballCamera.hpp>
#include <PartyKel/atb.hpp>

#include <vector>

static const Uint32 WINDOW_WIDTH = 1024;
static const Uint32 WINDOW_HEIGHT = 1024;

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
    
        glm::vec3 force = V * (v2-v1) / dt;
        return force;
}




// Structure permettant de simuler un drapeau à l'aide un système masse-ressort
struct Flag {
    int gridWidth, gridHeight; // Dimensions de la grille de points
    int width, height;
    glm::vec3 origin;
    glm::vec3 scale;

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

    // Crée un drapeau discretisé sous la forme d'une grille contenant gridWidth * gridHeight
    // points. Chaque point a pour masse mass / (gridWidth * gridHeight).
    // La taille du drapeau en 3D est spécifié par les paramètres width et height
    Flag(float mass, float width, float height, int gridWidth, int gridHeight):
        gridWidth(gridWidth), gridHeight(gridHeight), width(width), height(height),
        positionArray(gridWidth * gridHeight),
        velocityArray(gridWidth * gridHeight, glm::vec3(0.0f)),
        // massArray(gridWidth * gridHeight, mass / (gridWidth * gridHeight)),
        massArray(gridWidth * gridHeight, 10),
        forceArray(gridWidth * gridHeight, glm::vec3(0.f)), 
        origin(-0.5f * width, -0.5f * height, 0.f),
        scale(width / (gridWidth - 1), height / (gridHeight - 1), 1.f){
            
        for(int j = 0; j < gridHeight; ++j) {
            for(int i = 0; i < gridWidth; ++i) {
                int k = i + j * gridWidth;
                positionArray[k] = origin + glm::vec3(i, j, origin.z) * scale * 1.5f;
                // massArray[i + j * gridWidth] = 1 - ( i / (2*(gridHeight*gridWidth)));
                // massArray[i + j * gridWidth] = mass/ ((i*i+1)*(gridWidth*gridHeight));
                if(i==0) massArray[k] = 1000;
                else if(i==1) massArray[k] = 50;
                else massArray[k] = 50 / i; 
            }  
        }

        // Les longueurs à vide sont calculés à partir de la position initiale
        // des points sur le drapeau
        L0.x = scale.x;
        L0.y = scale.y;
        L1 = glm::length(L0);
        L2 = 2.f * L0;

        // Ces paramètres sont à fixer pour avoir un système stable: HAVE FUN !
        K0 = 0.01f;
        K1 = 0.01f;
        K2 = 0.01f;

        V0 = 0.1f;
        V1 = 0.1f;
        V2 = 0.1f;

        // K0 = 1.8;
        // K1 = 0.2;
        // K2 = 0.25;

        // V0 = 1.9f;
        // V1 = 1.3f;
        // V2 = 0.1f;

    }

    #define DEBUG 0
    // Applique les forces internes sur chaque point du drapeau SAUF les points fixes
    // TODO
    // Régler les masses
    void applyInternalForces(float dt) {
        int zero=0;
        // TOPOLOGIE 0

        // On applique la Hook Force et le Frein Cinétique sur chaque couple de particule
        for(int j = 0; j < gridHeight-1; ++j) {
            for(int i = 0; i < gridWidth-1; ++i) {

                int k = i + j * gridWidth;

                glm::vec3 HOOK_X = hookForce(K0, L0.x, positionArray[k], positionArray[k+1]);
                glm::vec3 HOOK_Y = hookForce(K0, L0.y, positionArray[k], positionArray[k+gridWidth]);

                glm::vec3 BRAKE_X = brakeForce(V0, dt, velocityArray[k], velocityArray[k+1]);
                glm::vec3 BRAKE_Y = brakeForce(V0, dt, velocityArray[k], velocityArray[k+gridWidth]);

                if(i>0){
                    forceArray[k] += HOOK_X + HOOK_Y + BRAKE_X + BRAKE_Y;
                }
                // else forceArray[k] += HOOK_Y + BRAKE_Y;
                forceArray[k+1] -= HOOK_X + BRAKE_X;
                forceArray[k+gridWidth] -= HOOK_Y + BRAKE_Y;
 
                if(DEBUG) std::cout << "TOPO 0: (" << k << " " << k+1 << ")" << " && (" << k << " " << k+gridWidth << ")" << std::endl;
            }
        }

        // dernière colonne
        for(int j = 0; j < gridHeight-1; ++j) {
            
            int k = j + (j+1) * (gridWidth-1);

            glm::vec3 HOOK_Y = hookForce(K0, L0.y, positionArray[k], positionArray[k+gridWidth]);
            glm::vec3 BRAKE_Y = brakeForce(V0, dt, velocityArray[k], velocityArray[k+gridWidth]);

            forceArray[k] += HOOK_Y + BRAKE_Y;
            forceArray[k + gridWidth] -= HOOK_Y + BRAKE_Y; 

            if(DEBUG) std::cout << "TOPO 0: (" << k << " " << k+gridWidth << ") last col" << std::endl; 
        }

        //dernière ligne 
        for(int i = 0; i < gridWidth-1; ++i){

            int k = i + (gridWidth) * (gridHeight-1);

            glm::vec3 HOOK_X = hookForce(K0, L0.x, positionArray[k], positionArray[k+1]);
            glm::vec3 BRAKE_X = brakeForce(V0, dt, velocityArray[k], velocityArray[k+1]);

            forceArray[k] += HOOK_X + BRAKE_X;
            forceArray[k+1] -= HOOK_X + BRAKE_X; 

            if(DEBUG) std::cout << "TOPO 0: (" << k << " " << k+1 << ") last line" << std::endl;
        }

        // dernière colonne et dernière ligne (la particule en bas à droite)
        // forceArray[gridHeight-2] += hookForce(K0, L0.y, positionArray[gridHeight-2], positionArray[gridHeight-1]);
        // forceArray[gridHeight-2] += brakeForce(V0, dt, velocityArray[gridHeight-2], velocityArray[gridHeight-1]);
        // forceArray[gridHeight-1] -= hookForce(K0, L0.y, positionArray[gridHeight-2], positionArray[gridHeight-1]);
        // forceArray[gridHeight-1] -= brakeForce(V0, dt, velocityArray[gridHeight-2], velocityArray[gridHeight-1]);
        // std::cout << "-->(" << gridHeight-2 << " " << gridHeight-1 << ")" << std::endl;



        // TOPOLOGIE 1

        for(int j = 0; j < gridHeight-1; ++j) {
            for(int i = 0; i < gridWidth-1; ++i) {

                int k = i + j * gridWidth;

                glm::vec3 HOOK_DIAGD = hookForce(K1, L1, positionArray[k], positionArray[k+1+gridWidth]);
                glm::vec3 HOOK_DIAGG = hookForce(K1, L1, positionArray[k], positionArray[k-1+gridWidth]);

                glm::vec3 BRAKE_DIAGD = brakeForce(V1, dt, velocityArray[k], velocityArray[k+1+gridWidth]);
                glm::vec3 BRAKE_DIAGG = brakeForce(V1, dt, velocityArray[k], velocityArray[k-1+gridWidth]);

                if(i>0) 
                    forceArray[k] += HOOK_DIAGD + HOOK_DIAGG + BRAKE_DIAGD + BRAKE_DIAGG;
                if(i>0)
                    forceArray[k-1+gridWidth] -= HOOK_DIAGG + BRAKE_DIAGG;

                forceArray[k+1+gridWidth] -= HOOK_DIAGD + BRAKE_DIAGD;

                if(i==0 && DEBUG) std::cout << "TOPO 1: " << "(" << k << " " << k+1+gridWidth << ") i>0" << std::endl;
                else if(DEBUG) std::cout << "TOPO 1: (" << k << " " << k+1+gridWidth << ")" << " && (" << k << " " << k-1+gridWidth << ")" << std::endl;

            }
        }

        // dernière colonne
        for(int j = 0; j < gridHeight-1; ++j) {
            
            int k = j + (j+1) * (gridWidth-1);

            glm::vec3 HOOK_DIAGG = hookForce(K1, L1, positionArray[k], positionArray[k-1+gridWidth]);
            glm::vec3 BRAKE_DIAGG = brakeForce(V1, dt, velocityArray[k], velocityArray[k-1+gridWidth]);

            forceArray[k] += HOOK_DIAGG + BRAKE_DIAGG;
            forceArray[k-1+gridWidth] -= HOOK_DIAGG + BRAKE_DIAGG;

            if(DEBUG) std::cout << "TOPO 1: (" << k << " " << k-1+gridWidth << ") last col" << std::endl; 
        }



        // TOPOLOGIE 2

        for(int j = 0; j < gridHeight-2; ++j) {
            for(int i = 0; i < gridWidth-2; ++i) {

                int k = i + j * gridWidth;

                glm::vec3 HOOK_X2 = hookForce(K2, L2.x, positionArray[k], positionArray[k+2]);
                glm::vec3 HOOK_Y2 = hookForce(K2, L2.y, positionArray[k], positionArray[k+2*gridWidth]);

                glm::vec3 BRAKE_X2 = brakeForce(V2, dt, velocityArray[k], velocityArray[k+2]);
                glm::vec3 BRAKE_Y2 = brakeForce(V2, dt, velocityArray[k], velocityArray[k+2*gridWidth]);

                // if(i>0) 
                    forceArray[k] += HOOK_X2 + HOOK_Y2 + BRAKE_X2 + BRAKE_Y2;
                
                forceArray[k+2] -= HOOK_X2 + BRAKE_X2;
                forceArray[k+2*gridWidth] -= HOOK_Y2 + BRAKE_Y2;
                
                // if(i==0) std::cout "";
                if(DEBUG) std::cout << "TOPO 2: (" << k << " " << k+2 << ")" << " && (" << k << " " << k+2*gridWidth << ")" << std::endl;
            }
        }

        // dernières colonnes (Y only)
        for(int j = 0; j < gridHeight-2; ++j) {

            int k = j + (j+1) * (gridWidth-1);

            glm::vec3 HOOK_Y2 = hookForce(K2, L2.y, positionArray[k], positionArray[k+2*gridWidth]);
            glm::vec3 BRAKE_Y2 = brakeForce(V2, dt, velocityArray[k], velocityArray[k+2*gridWidth]);
            forceArray[k] += HOOK_Y2 + BRAKE_Y2;
            forceArray[k+2*gridWidth] -= HOOK_Y2 + BRAKE_Y2; 
            if(DEBUG) std::cout << "TOPO 2: (" << k << " " << k+2*gridWidth << ") last col" << std::endl; 

            HOOK_Y2 = hookForce(K2, L2.y, positionArray[k-1], positionArray[k+2*gridWidth-1]);
            BRAKE_Y2 = brakeForce(V2, dt, velocityArray[k-1], velocityArray[k+2*gridWidth-1]);
            forceArray[k-1] += HOOK_Y2 + BRAKE_Y2;
            forceArray[k+2*gridWidth-1] -= HOOK_Y2 + BRAKE_Y2; 
            if(DEBUG) std::cout << "TOPO 2: (" << k-1 << " " << k+2*gridWidth-1 << ") penultimate col" << std::endl; 
        }

        //dernières lignes (X only)
        for(int i = 0; i < gridWidth-2; ++i){

            int k = i + (gridWidth) * (gridHeight-1);

            glm::vec3 HOOK_X2 = hookForce(K2, L2.x, positionArray[k], positionArray[k+2]);
            glm::vec3 BRAKE_X2 = brakeForce(V2, dt, velocityArray[k], velocityArray[k+2]);
            forceArray[k] += HOOK_X2 + BRAKE_X2;
            forceArray[k+2] -= HOOK_X2 + BRAKE_X2; 
            if(DEBUG) std::cout << "TOPO 2: (" << k << " " << k+2 << ") last line" << std::endl;

            HOOK_X2 = hookForce(K2, L2.x, positionArray[k-gridWidth], positionArray[k+2-gridWidth]);
            BRAKE_X2 = brakeForce(V2, dt, velocityArray[k-gridWidth], velocityArray[k+2-gridWidth]);
            forceArray[k-gridWidth] += HOOK_X2 + BRAKE_X2;
            forceArray[k+2-gridWidth] -= HOOK_X2 + BRAKE_X2; 
            if(DEBUG) std::cout << "TOPO 2: (" << k-gridWidth << " " << k+2-gridWidth << ") penultimate line" << std::endl;
        }


    }

    // Applique une force externe sur chaque point du drapeau SAUF les points fixes
    void applyExternalForce(const glm::vec3& F) {
    
        for(int j = 0; j < gridHeight; ++j) {
            for(int i = 0; i < gridWidth; ++i) {
                int k = i + j * gridWidth;
                // if(k%gridWidth!=0){
                if(i!=0){
                    forceArray[k] += F;     
                }
            }
        }


    }

    void reset(){

        for(int j = 0; j < gridHeight; ++j) {
            for(int i = 0; i < gridWidth; ++i) {
                int k = i + j * gridWidth;
                positionArray[k] = origin + glm::vec3(i, j, origin.z) * scale * 1.5f;
                if(i==0) massArray[k] = 1000;
                else if(i==1) massArray[k] = 50;
                else massArray[k] = 50 / i; 
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

        // on reset les forces à 0
        for(int s=0; s < forceArray.size(); ++s){
            forceArray[s] = glm::vec3(0.f);
        }

    }


};

int main() {
    WindowManager wm(WINDOW_WIDTH, WINDOW_HEIGHT, "Fun with Flags");
    wm.setFramerate(30);

    // Initialisation de AntTweakBar (pour la GUI)
    TwInit(TW_OPENGL, NULL);
    TwWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);

    Flag flag(4096.f, 2, 1.5, 6, 4); // Création d'un drapeau // Flag(float mass, float width, float height, int gridWidth, int gridHeight)
    glm::vec3 GRAVITY(0.00f, -0.01f, 0.f); // Gravity
    glm::vec3 WIND = glm::sphericalRand(0.001f); // 0.001f
    // WIND = glm::vec3(0.0,0.0,0.1);

    FlagRenderer3D renderer(flag.gridWidth, flag.gridHeight);
    renderer.setProjMatrix(glm::perspective(70.f, float(WINDOW_WIDTH) / WINDOW_HEIGHT, 0.1f, 10000.f));

    TrackballCamera camera;
    int mouseLastX, mouseLastY;

    // Temps s'écoulant entre chaque frame
    float dt = 0.f;

    bool done = false;
    bool wireframe = true;

    // GUI
    TwBar* gui = TwNewBar("Parametres");

    atb::addVarRW(gui, ATB_VAR(flag.L0.x), "step=0.01");
    atb::addVarRW(gui, ATB_VAR(flag.L0.y), "step=0.01");
    atb::addVarRW(gui, ATB_VAR(flag.L1), "step=0.01");
    atb::addVarRW(gui, ATB_VAR(flag.L2.x), "step=0.01");
    atb::addVarRW(gui, ATB_VAR(flag.L2.y), "step=0.01");
    atb::addVarRW(gui, ATB_VAR(flag.K0), "step=0.1");
    atb::addVarRW(gui, ATB_VAR(flag.K1), "step=0.1");
    atb::addVarRW(gui, ATB_VAR(flag.K2), "step=0.1");
    atb::addVarRW(gui, ATB_VAR(flag.V0), "step=0.1");
    atb::addVarRW(gui, ATB_VAR(flag.V1), "step=0.1");
    atb::addVarRW(gui, ATB_VAR(flag.V2), "step=0.1");
    atb::addButton(gui, "reset", [&]() {
        renderer.clear(); 
        renderer.setViewMatrix(camera.getViewMatrix());
        // renderer.setProjMatrix(glm::perspective(70.f, float(WINDOW_WIDTH) / WINDOW_HEIGHT, 0.1f, 10000.f));
        renderer.drawGrid(flag.positionArray.data(), wireframe);

        flag.reset();
    });

    while(!done) {
        wm.startMainLoop();

        // Rendu
        renderer.clear();

        renderer.setViewMatrix(camera.getViewMatrix());
        renderer.drawGrid(flag.positionArray.data(), wireframe);

        

        // Simulation
        if(dt > 0.f) {
            flag.applyExternalForce(GRAVITY); // Applique la gravité
            flag.applyExternalForce(WIND); // Applique un "vent" de direction aléatoire et de force 0.1 Newtons
            flag.applyInternalForces(dt); // Applique les forces internes
            flag.update(dt); // Mise à jour du système à partir des forces appliquées
        }

        TwDraw();

        // Gestion des evenements
		SDL_Event e;
        while(wm.pollEvent(e)) {
            int handled = TwEventSDL(&e, SDL_MAJOR_VERSION, SDL_MINOR_VERSION);

			if(!handled){
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
