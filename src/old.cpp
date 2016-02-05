 for(int j = 0; j < gridHeight-1; ++j) {
            for(int i = 1; i < gridWidth-1; ++i) {
                int k = i + j * gridWidth;
                positionArray[k] += hookForce(K0, L0.x, positionArray[k], positionArray[k + 1]) + hookForce(K0, L0.y, positionArray[k], positionArray[k + gridWidth]);
                positionArray[k+1] -= hookForce(K0, L0.x, positionArray[k], positionArray[k + 1]);
                positionArray[k + gridWidth] -= hookForce(K0, L0.y, positionArray[k], positionArray[k + gridWidth]);

                positionArray[k] += brakeForce(V0, dt, velocityArray[k], velocityArray[k+1]) + brakeForce(V0, dt, velocityArray[k], velocityArray[k+gridWidth]);
                positionArray[k+1] -= brakeForce(V0, dt, velocityArray[k], velocityArray[k+1]);
                positionArray[k+gridWidth] -= brakeForce(V0, dt, velocityArray[k], velocityArray[k+gridWidth]);

            }
        }
        // première colonne
        for(int j = 0; j < gridHeight; ++j) {
            int k = j;
            // positionArray[k] += hookForce(K0, L0.y, positionArray[k], positionArray[k + 1]);
            positionArray[k + 1] -= hookForce(K0, L0.y, positionArray[k], positionArray[k + 1]);

            positionArray[k + 1] -= brakeForce(V0, dt, velocityArray[k], velocityArray[k+1]);
        }
        // dernière colonne
        for(int j = 0; j < gridHeight-1; ++j) {
            int k = j * gridWidth;
            positionArray[k] += hookForce(K0, L0.y, positionArray[k], positionArray[k + gridWidth]);
            positionArray[k + gridWidth] -= hookForce(K0, L0.y, positionArray[k], positionArray[k + gridWidth]);

            positionArray[k] += brakeForce(V0, dt, velocityArray[k], velocityArray[k+gridWidth]);
            positionArray[k + gridWidth] -= brakeForce(V0, dt, velocityArray[k], velocityArray[k + gridWidth]);

        }
        // dernière colonne et dernière ligne (la particule en bas à droite)
        positionArray[gridHeight-2] += hookForce(K0, L0.y, positionArray[gridHeight-2], positionArray[gridHeight-1]);
        positionArray[gridHeight-1] -= hookForce(K0, L0.y, positionArray[gridHeight-2], positionArray[gridHeight-1]);

        positionArray[gridHeight-2] += brakeForce(V0, dt, velocityArray[gridHeight-2], velocityArray[gridHeight-1]);
        positionArray[gridHeight-1] -= brakeForce(V0, dt, velocityArray[gridHeight-2], velocityArray[gridHeight-1]);