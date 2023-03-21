//---------------------------------------------------------------------
//
// Copyright 2007, SensAble Technologies, Inc.
//
// File: Model_3DS.h
//
// Author: Venkat Gourishankar	
//
// Created: 10/02/2007
//
// Description: Declares the Model_3DS class for loading 3DS models
// This is a simple class for loading and viewing 3D Studio model files (.3ds). 
// Original Author: Matthew Fairfax
// http://www.garagegames.com/index.php?sec=mg&mod=resource&page=view&qid=506
// I have modified this file to suit the project purposes
// Functionalities that I added:
// - Removed Binding with GLTexture lib so that anyone can use thier own texture loading lib
// - Can display separate objects based on object index list
// - Finding Bound Box for models 
// - Setting up Camera based on Bound Box of models (X, Y, Z, ISO cameras)
// - Setting up haptic material properties to models
// - Setting up haptic rendering for models depending on model properties
//
//---------------------------------------------------------------------
#pragma once

#ifndef MODEL_3DS_H
#define MODEL_3DS_H

#include "Globals.h"
#include <stdio.h>

class Model_3DS
{
public:
	// m_A VERY simple vector struct
	// I could have included a complex class but I wanted the model class to stand alone
	struct Vector {
		float x;
		float y;
		float z;
	};

	// m_Vertex struct to make code easier to read in places
	struct m_Vertex {
		float x;
		float y;
		float z;
	};

	// Color struct holds the diffuse color of the material
	struct Color4i {
		unsigned char r;
		unsigned char g;
		unsigned char b;
		unsigned char a;
	};

	// Holds the material info
	// TODO: add color support for non textured polys
	struct Material {
		char name[80];	// The material's name
		char TextureFileName[1024];
//		GLTexture tex;	// The texture (this is the only outside reference in this class)
		bool textured;	// whether or not it is textured
		Color4i color;
	};

	// Every chunk in the 3ds file starts with this struct
	struct ChunkHeader {
		unsigned short id;	// The chunk's id
        int  len;	// The lenght of the chunk
	};

	// I sort the mesh by material so that I won't have to switch m_textures a great deal
	struct MaterialFaces {
		unsigned short *subFaces;	// m_Index to our vertex array of all the faces that use this material
		int numSubFaces;			// The number of faces
		int MatIndex;				// An index to our materials
	};

	// The 3ds file can be made up of several objects
	struct Object {
		char name[80];				// The object name
		
		float *Vertexes;			// The array of vertices
		float *Normals;				// The array of the normals for the vertices
		float *TexCoords;			// The array of texture coordinates for the vertices
		unsigned short *Faces;		// The array of face indices
		int numFaces;				// The number of faces
		int numMatFaces;			// The number of differnet material faces
		int numVerts;				// The number of vertices
		int numTexCoords;			// The number of vertices
		bool textured;				// True: the object has m_textures
		MaterialFaces *MatFaces;	// The faces are divided by materials
		Vector pos;					// The position to move the object to
		Vector rot;					// The angles to rotate the object
        double transform[16];
	};
	struct Camera {
		float CamPos[3];
		float CamTgt[3];
		float CamRoll;
		float CamFov;
		bool CamSeeCone;
		float CamNearRange;
		float CamFarRange;
	};

	char TexName[80];			// m_Texture file name
	char *modelname;		// The name of the model
	char *path;				// The path of the model
	int numObjects;			// Total number of objects in the model
	int numMaterials;		// Total number of materials in the model
	int totalVerts;			// Total number of vertices in the model
	int totalFaces;			// Total number of faces in the model
	bool shownormals;		// True: show the normals
	Material *Materials;	// The array of materials
	Object *Objects;		// The array of objects in the model
	Camera Cam;				// Camera Object - Can be X, Y, Z or ISO
	Vector pos;				// The position to move the model to
	Vector rot;				// The angles to rotate the model
	Vector minXYZ;			// The min end of BoundBox
	Vector maxXYZ;			// The max end of BoundBox
	float scale;			// The size you want the model scaled to
	bool lit;				// True: the model is lit
	bool visible;			// True: the model gets rendered
	void Load(char *name);	// Loads a model
	void draw();			// Draws the model

    void DrawWithTransform();			// Draws the model
	void DrawObject(int objectNum);	//Draws specific object of the model
	// Draws the Bound Box of the Object
	void DrawBoundBox();
	// Calculates the Bound Box of the Object and draws it 
	void CalcBoundBox();
	void ShowAxes();
	FILE *bin3ds;			// The binary 3ds file
	Model_3DS();			// Constructor
	virtual ~Model_3DS();	// Destructor

private:

	
    void IntColorChunkProcessor(int length, int findex, int matindex);
    void FloatColorChunkProcessor(int length, int findex, int matindex);
	// Processes the Main Chunk that all the other chunks exist is
    void MainChunkProcessor(int length, int findex);
		// Processes the model's info
        void EditChunkProcessor(int length, int findex);
			
			// Processes the model's materials
            void MaterialChunkProcessor(int length, int findex, int matindex);
				// Processes the names of the materials
                void MaterialNameChunkProcessor(int length, int findex, int matindex);
				// Processes the material's diffuse color
                void DiffuseColorChunkProcessor(int length, int findex, int matindex);
				// Processes the material's texture maps
                void TextureMapChunkProcessor(int length, int findex, int matindex);
					// Processes the names of the m_textures and load the m_textures
                    void MapNameChunkProcessor(int length, int findex, int matindex);
			
			// Processes the model's geometry
            void ObjectChunkProcessor(int length, int findex, int objindex);
				// Processes the triangles of the model
                void TriangularMeshChunkProcessor(int length, int findex, int objindex);
					// Processes the vertices of the model and loads them
                    void VertexListChunkProcessor(int length, int findex, int objindex);
					// Processes the texture cordiantes of the vertices and loads them
                    void TexCoordsChunkProcessor(int length, int findex, int objindex);
					// Processes the faces of the model and loads the faces
                    void FacesDescriptionChunkProcessor(int length, int findex, int objindex);
						// Processes the materials of the faces and splits them up by material
                        void FacesMaterialsListChunkProcessor(int length, int findex, int objindex, int subfacesindex);
						//Processes the Camera of the objects
						
	// Calculates the normals of the vertices by averaging
	// the normals of the faces that use that vertex
	void CalculateNormals();

	
};

#endif MODEL_3DS_H
