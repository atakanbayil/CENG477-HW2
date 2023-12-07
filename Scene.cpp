#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>

#include "tinyxml2.h"
#include "Triangle.h"
#include "Helpers.h"
#include "Scene.h"

using namespace tinyxml2;
using namespace std;

/*
	Parses XML file
*/
Matrix4 Scene::getResultingTransformationMatrix(Mesh mesh, Vec4 v){
    Matrix4 result = getIdentityMatrix();
    for (int i = 0; i < mesh.transformationTypes.size() ; i++){
        if (mesh.transformationTypes[i] == 't'){
            Translation translation = *this->translations[mesh.transformationIds[i]-1];
            result = multiplyMatrixWithMatrix(translation.getTranslationMatrix(), result);
        }
        if (mesh.transformationTypes[i] == 's'){
            Scaling scaling = *this->scalings[mesh.transformationIds[i]-1];
            result = multiplyMatrixWithMatrix(scaling.getScalingMatrix(v), result);
        }
        if (mesh.transformationTypes[i] == 'r'){
            Rotation rotation = *this->rotations[mesh.transformationIds[i]-1];
            result = multiplyMatrixWithMatrix(rotation.getRotationMatrix(), result);
        }
    }
    return result;
}

Vec3 Scene::getBarycentricCoordinates(Vec3 point, Vec3 firstVertex, Vec3 secondVertex, Vec3 thirdVertex) {
    Vec3 result;
    double alphaNom = (point.x*(secondVertex.y-thirdVertex.y)+ point.y*(thirdVertex.x- secondVertex.x)+(secondVertex.x*thirdVertex.y)-(secondVertex.y*thirdVertex.x));
    double alphaDenom = (firstVertex.x*(secondVertex.y-thirdVertex.y)+ firstVertex.y*(thirdVertex.x- secondVertex.x)+(secondVertex.x*thirdVertex.y)-(secondVertex.y*thirdVertex.x));
    double alpha = alphaNom/alphaDenom;
    double betaNom = (point.x*(thirdVertex.y-firstVertex.y)+ point.y*(firstVertex.x- thirdVertex.x)+(thirdVertex.x*firstVertex.y)-(thirdVertex.y*firstVertex.x));
    double betaDenom = (secondVertex.x*(thirdVertex.y-firstVertex.y)+ secondVertex.y*(firstVertex.x- thirdVertex.x)+(thirdVertex.x*firstVertex.y)-(thirdVertex.y*firstVertex.x));
    double beta = betaNom/betaDenom;
    double gamma = 1.0 - alpha - beta;
    result.x = alpha;
    result.y = beta;
    result.z = gamma;
    return result;
}
/*
	Transformations, clipping, culling, rasterization are done here.
	You may define helper functions.
*/


bool Scene::clipper (Vec4 &v1, Vec4 &v2, Vec3 &c1, Vec3 &c2){
    double te = 0.0;
    double tl = 1.0;
    double dx = v2.x - v1.x;
    double dy = v2.y - v1.y;
    double dz = v2.z - v1.z;
    Vec3 dc = subtractVec3(c2, c1);
    if (!isVisible(dx, (-1.0-(v1.x)), te, tl))
        return false;
    if (!isVisible(-dx, ((v1.x)-1.0), te, tl))
        return false;
    if (!isVisible(dy, (-1.0-(v1.y)), te, tl))
        return false;
    if (!isVisible(-dy, ((v1.y)-1.0), te, tl))
        return false;
    if (!isVisible(dz, (-1.0-(v1.z)), te, tl))
        return false;
    if (!isVisible(-dz, ((v1.z)-1.0), te, tl))
        return false;
    if (tl<1.0) {
        v2.x = v1.x + tl * dx;
        v2.y = v1.y + tl * dy;
        v2.z = v1.z + tl * dz;
        c2 = addVec3(c1, multiplyVec3WithScalar(dc, tl));
    }
    if (te>0.0) {
        v1.x = v1.x + te * dx;
        v1.y = v1.y + te * dy;
        v1.z = v1.z + te * dz;
        c1 = addVec3(c1, multiplyVec3WithScalar(dc, te));
    }
    return true;

}


void Scene::forwardRenderingPipeline(Camera *camera)
{
    Matrix4 viewportMatrix = camera->getViewportTransformationMatrix();

    Matrix4 cameraTransformationMatrix = camera->getCameraTransformationMatrix();
    Matrix4 perspectiveMatrix = camera->getProjectionTransformationMatrix(camera->projectionType);

    Matrix4 projectionMatrix = multiplyMatrixWithMatrix(perspectiveMatrix, cameraTransformationMatrix);
    //projectionMatrix = multiplyMatrixWithMatrix(projectionMatrix, cameraTransformationMatrix);

    // Viewport x_min?, rotation

    for (int i = 0; i < this->meshes.size(); i++)
    {
        Mesh mesh = *this->meshes[i];


        for (int j = 0; j < mesh.numberOfTriangles; j++){
            Triangle triangle = mesh.triangles[j];
            Vec4 firstVertex = Vec4(this->vertices[triangle.getFirstVertexId()-1]->x, this->vertices[triangle.getFirstVertexId()-1]->y, this->vertices[triangle.getFirstVertexId()-1]->z, 1,
                                    this->vertices[triangle.getFirstVertexId()-1]->colorId);
            Vec4 secondVertex = Vec4(this->vertices[triangle.getSecondVertexId()-1]->x, this->vertices[triangle.getSecondVertexId()-1]->y, this->vertices[triangle.getSecondVertexId()-1]->z, 1,
                                    this->vertices[triangle.getSecondVertexId()-1]->colorId);
            Vec4 thirdVertex = Vec4(this->vertices[triangle.getThirdVertexId()-1]->x, this->vertices[triangle.getThirdVertexId()-1]->y, this->vertices[triangle.getThirdVertexId()-1]->z, 1,
                                    this->vertices[triangle.getThirdVertexId()-1]->colorId);

            Matrix4 transformMatrixV1 = getResultingTransformationMatrix(mesh, firstVertex);
            transformMatrixV1 = multiplyMatrixWithMatrix(projectionMatrix, transformMatrixV1);

            Matrix4 transformMatrixV2 = getResultingTransformationMatrix(mesh, secondVertex);
            transformMatrixV2 = multiplyMatrixWithMatrix(projectionMatrix, transformMatrixV2);

            Matrix4 transformMatrixV3 = getResultingTransformationMatrix(mesh, thirdVertex);
            transformMatrixV3 = multiplyMatrixWithMatrix(projectionMatrix, transformMatrixV3);

            Vec4 v1 = multiplyMatrixWithVec4(transformMatrixV1, firstVertex);
            Vec4 v2 = multiplyMatrixWithVec4(transformMatrixV2, secondVertex);
            Vec4 v3 = multiplyMatrixWithVec4(transformMatrixV3, thirdVertex);

            v1 = multiplyVec4WithScalar(v1, 1.0/v1.t);
            v2 = multiplyVec4WithScalar(v2, 1.0/v2.t);
            v3 = multiplyVec4WithScalar(v3, 1.0/v3.t);


            // Culling
            if (this->cullingEnabled){
                Vec3 normal = getNormalOfTriangle(convertVec4ToVec3(v1), convertVec4ToVec3(v2), convertVec4ToVec3(v3));
                if (dotProductVec3(normal, convertVec4ToVec3(v1)) < 0){
                    continue;
                }
            }


            if (mesh.type == 1){ // for solid meshes
                v1 = multiplyMatrixWithVec4(viewportMatrix, v1);
                v2 = multiplyMatrixWithVec4(viewportMatrix, v2);
                v3 = multiplyMatrixWithVec4(viewportMatrix, v3);
                int minX = min(v1.x, min(v2.x, v3.x));
                int maxX = max(v1.x, max(v2.x, v3.x));
                if (maxX <= 0) continue;
                if (maxX >= camera->horRes) maxX = camera->horRes-1;
                int minY = min(v1.y, min(v2.y, v3.y));
                int maxY = max(v1.y, max(v2.y, v3.y));
                if (maxY <= 0) continue;
                if (maxY >= camera->verRes) maxY = camera->verRes-1;
                for (int x = max(minX,0); x <= maxX; x++){
                    for (int y = max(minY,0); y <= maxY; y++){
                        Vec3 barycentricCoordinates = getBarycentricCoordinates(Vec3(x, y, 0, -1), Vec3(v1.x, v1.y, v1.z, firstVertex.colorId),
                                                                                Vec3(v2.x, v2.y, v2.z, secondVertex.colorId), Vec3(v3.x, v3.y, v3.z, thirdVertex.colorId));
                        if (barycentricCoordinates.x >= 0.0 && barycentricCoordinates.y >= 0.0 && barycentricCoordinates.z >= 0.0){
                            Vec3 firstVertexColor(this->colorsOfVertices[firstVertex.colorId-1]->r, this->colorsOfVertices[firstVertex.colorId-1]->g, this->colorsOfVertices[firstVertex.colorId-1]->b, -1);
                            Vec3 secondVertexColor(this->colorsOfVertices[secondVertex.colorId-1]->r, this->colorsOfVertices[secondVertex.colorId-1]->g, this->colorsOfVertices[secondVertex.colorId-1]->b, -1);
                            Vec3 thirdVertexColor(this->colorsOfVertices[thirdVertex.colorId-1]->r, this->colorsOfVertices[thirdVertex.colorId-1]->g, this->colorsOfVertices[thirdVertex.colorId-1]->b, -1);

                            Vec3 pixelColor = addVec3(multiplyVec3WithScalar(firstVertexColor, barycentricCoordinates.x), multiplyVec3WithScalar(secondVertexColor, barycentricCoordinates.y));
                            pixelColor = addVec3(pixelColor, multiplyVec3WithScalar(thirdVertexColor, barycentricCoordinates.z));

                            this->image[x][y].r = round(pixelColor.x);
                            this->image[x][y].g = round(pixelColor.y);
                            this->image[x][y].b = round(pixelColor.z);
                        }
                    }
                }
            }
            else if (mesh.type == 0){ // for wireframe meshes

                Vec4 modifiedVertices[] = {v1, v2, v3};
                for (int b = 0 ; b < 2; b++) {
                    for (int a = b + 1; a < 3; a++) {
                        Vec4 vertex_0 = modifiedVertices[b];
                        Vec4 vertex_1 = modifiedVertices[a];
                        Vec3 color0 = Vec3(this->colorsOfVertices[vertex_0.colorId - 1]->r,
                                           this->colorsOfVertices[vertex_0.colorId - 1]->g,
                                           this->colorsOfVertices[vertex_0.colorId - 1]->b, -1);
                        Vec3 color1 = Vec3(this->colorsOfVertices[vertex_1.colorId - 1]->r,
                                           this->colorsOfVertices[vertex_1.colorId - 1]->g,
                                           this->colorsOfVertices[vertex_1.colorId - 1]->b, -1);
                        if (!this->clipper(vertex_0, vertex_1, color0, color1)) {
                            continue;
                        }
                        vertex_0 = multiplyMatrixWithVec4(viewportMatrix, vertex_0);
                        vertex_1 = multiplyMatrixWithVec4(viewportMatrix, vertex_1);
                        double slope = (vertex_1.y - vertex_0.y) / (vertex_1.x - vertex_0.x);
                        if (slope > 0 && slope < 1){
                            int y = min(vertex_0.y, vertex_1.y);
                            int d = 2* abs(vertex_1.y - vertex_0.y) - abs(vertex_1.x - vertex_0.x);
                            int xmax = max(vertex_0.x, vertex_1.x);
                            for ( int x = min(vertex_0.x, vertex_1.x); x <= xmax; x++) {
                                image[x][y].r = round(color0.x * abs(vertex_1.x - x) / abs(vertex_1.x - vertex_0.x) + color1.x * abs(vertex_0.x - x) / abs(vertex_1.x - vertex_0.x));
                                image[x][y].g = round(color0.y * abs(vertex_1.x - x) / abs(vertex_1.x - vertex_0.x) + color1.y * abs(vertex_0.x - x) / abs(vertex_1.x - vertex_0.x));
                                image[x][y].b = round(color0.z * abs(vertex_1.x - x) / abs(vertex_1.x - vertex_0.x) + color1.z * abs(vertex_0.x - x) / abs(vertex_1.x - vertex_0.x));
                                if (d <= 0){
                                    d += 2*abs(vertex_1.y - vertex_0.y);
                                }
                                else{
                                    d += 2*abs(vertex_1.y - vertex_0.y) - 2*abs(vertex_1.x - vertex_0.x);
                                    y++;
                                }
                            }
                        }
                        else if (slope > 1){
                            int x = min(vertex_0.x, vertex_1.x);
                            int d = 2* abs(vertex_1.x - vertex_0.x) - abs(vertex_1.y - vertex_0.y);
                            int ymax = max(vertex_0.y, vertex_1.y);
                            for ( int y = min(vertex_0.y, vertex_1.y); y <= ymax; y++) {
                                image[x][y].r = round(color0.x * abs(vertex_1.y - y) / abs(vertex_1.y - vertex_0.y) + color1.x * abs(vertex_0.y - y) / abs(vertex_1.y - vertex_0.y));
                                image[x][y].g = round(color0.y * abs(vertex_1.y - y) / abs(vertex_1.y - vertex_0.y) + color1.y * abs(vertex_0.y - y) / abs(vertex_1.y - vertex_0.y));
                                image[x][y].b = round(color0.z * abs(vertex_1.y - y) / abs(vertex_1.y - vertex_0.y) + color1.z * abs(vertex_0.y - y) / abs(vertex_1.y - vertex_0.y));
                                if (d <= 0){
                                    d += 2*abs(vertex_1.x - vertex_0.x);
                                }
                                else{
                                    d += 2*abs(vertex_1.x - vertex_0.x) - 2*abs(vertex_1.y - vertex_0.y);
                                    x++;
                                }
                            }
                        }
                        else if (slope <= 0 && slope >= -1){
                            int y = max(vertex_0.y, vertex_1.y);
                            int d = 2* abs(vertex_1.y - vertex_0.y) - abs(vertex_1.x - vertex_0.x);
                            int xmax = max(vertex_0.x, vertex_1.x);
                            for ( int x = min(vertex_0.x, vertex_1.x); x <= xmax; x++) {
                                image[x][y].r = round(color0.x * abs(vertex_1.x - x) / abs(vertex_1.x - vertex_0.x) + color1.x * abs(vertex_0.x - x) / abs(vertex_1.x - vertex_0.x));
                                image[x][y].g = round(color0.y * abs(vertex_1.x - x) / abs(vertex_1.x - vertex_0.x) + color1.y * abs(vertex_0.x - x) / abs(vertex_1.x - vertex_0.x));
                                image[x][y].b = round(color0.z * abs(vertex_1.x - x) / abs(vertex_1.x - vertex_0.x) + color1.z * abs(vertex_0.x - x) / abs(vertex_1.x - vertex_0.x));
                                if (d <= 0){
                                    d += 2*abs(vertex_1.y - vertex_0.y);
                                }
                                else{
                                    d += 2*abs(vertex_1.y - vertex_0.y) - 2*abs(vertex_1.x - vertex_0.x);
                                    y--;
                                }
                            }
                        }
                        else if (slope < -1) {
                            int x = max(vertex_0.x, vertex_1.x);
                            int d = 2* abs(vertex_1.x - vertex_0.x) - abs(vertex_1.y - vertex_0.y);
                            int ymax = max(vertex_0.y, vertex_1.y);
                            for ( int y = min(vertex_0.y, vertex_1.y); y <= ymax; y++) {
                                image[x][y].r = round(color0.x * abs(vertex_1.y - y) / abs(vertex_1.y - vertex_0.y) + color1.x * abs(vertex_0.y - y) / abs(vertex_1.y - vertex_0.y));
                                image[x][y].g = round(color0.y * abs(vertex_1.y - y) / abs(vertex_1.y - vertex_0.y) + color1.y * abs(vertex_0.y - y) / abs(vertex_1.y - vertex_0.y));
                                image[x][y].b = round(color0.z * abs(vertex_1.y - y) / abs(vertex_1.y - vertex_0.y) + color1.z * abs(vertex_0.y - y) / abs(vertex_1.y - vertex_0.y));
                                if (d <= 0){
                                    d += 2*abs(vertex_1.x - vertex_0.x);
                                }
                                else{
                                    d += 2*abs(vertex_1.x - vertex_0.x) - 2*abs(vertex_1.y - vertex_0.y);
                                    x--;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *xmlElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *rootNode = xmlDoc.FirstChild();

	// read background color
	xmlElement = rootNode->FirstChildElement("BackgroundColor");
	str = xmlElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	xmlElement = rootNode->FirstChildElement("Culling");
	if (xmlElement != NULL)
	{
		str = xmlElement->GetText();

		if (strcmp(str, "enabled") == 0)
		{
			this->cullingEnabled = true;
		}
		else
		{
			this->cullingEnabled = false;
		}
	}

	// read cameras
	xmlElement = rootNode->FirstChildElement("Cameras");
	XMLElement *camElement = xmlElement->FirstChildElement("Camera");
	XMLElement *camFieldElement;
	while (camElement != NULL)
	{
		Camera *camera = new Camera();

		camElement->QueryIntAttribute("id", &camera->cameraId);

		// read projection type
		str = camElement->Attribute("type");

		if (strcmp(str, "orthographic") == 0)
		{
			camera->projectionType = ORTOGRAPHIC_PROJECTION;
		}
		else
		{
			camera->projectionType = PERSPECTIVE_PROJECTION;
		}

		camFieldElement = camElement->FirstChildElement("Position");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf", &camera->position.x, &camera->position.y, &camera->position.z);

		camFieldElement = camElement->FirstChildElement("Gaze");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf", &camera->gaze.x, &camera->gaze.y, &camera->gaze.z);

		camFieldElement = camElement->FirstChildElement("Up");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf", &camera->v.x, &camera->v.y, &camera->v.z);

		camera->gaze = normalizeVec3(camera->gaze);
		camera->u = crossProductVec3(camera->gaze, camera->v);
		camera->u = normalizeVec3(camera->u);

		camera->w = inverseVec3(camera->gaze);
		camera->v = crossProductVec3(camera->u, camera->gaze);
		camera->v = normalizeVec3(camera->v);

		camFieldElement = camElement->FirstChildElement("ImagePlane");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &camera->left, &camera->right, &camera->bottom, &camera->top,
			   &camera->near, &camera->far, &camera->horRes, &camera->verRes);

		camFieldElement = camElement->FirstChildElement("OutputName");
		str = camFieldElement->GetText();
		camera->outputFilename = string(str);

		this->cameras.push_back(camera);

		camElement = camElement->NextSiblingElement("Camera");
	}

	// read vertices
	xmlElement = rootNode->FirstChildElement("Vertices");
	XMLElement *vertexElement = xmlElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (vertexElement != NULL)
	{
		Vec3 *vertex = new Vec3();
		Color *color = new Color();

		vertex->colorId = vertexId;

		str = vertexElement->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = vertexElement->Attribute("color");
		sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

		this->vertices.push_back(vertex);
		this->colorsOfVertices.push_back(color);

		vertexElement = vertexElement->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	xmlElement = rootNode->FirstChildElement("Translations");
	XMLElement *translationElement = xmlElement->FirstChildElement("Translation");
	while (translationElement != NULL)
	{
		Translation *translation = new Translation();

		translationElement->QueryIntAttribute("id", &translation->translationId);

		str = translationElement->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		this->translations.push_back(translation);

		translationElement = translationElement->NextSiblingElement("Translation");
	}

	// read scalings
	xmlElement = rootNode->FirstChildElement("Scalings");
	XMLElement *scalingElement = xmlElement->FirstChildElement("Scaling");
	while (scalingElement != NULL)
	{
		Scaling *scaling = new Scaling();

		scalingElement->QueryIntAttribute("id", &scaling->scalingId);
		str = scalingElement->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		this->scalings.push_back(scaling);

		scalingElement = scalingElement->NextSiblingElement("Scaling");
	}

	// read rotations
	xmlElement = rootNode->FirstChildElement("Rotations");
	XMLElement *rotationElement = xmlElement->FirstChildElement("Rotation");
	while (rotationElement != NULL)
	{
		Rotation *rotation = new Rotation();

		rotationElement->QueryIntAttribute("id", &rotation->rotationId);
		str = rotationElement->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		this->rotations.push_back(rotation);

		rotationElement = rotationElement->NextSiblingElement("Rotation");
	}

	// read meshes
	xmlElement = rootNode->FirstChildElement("Meshes");

	XMLElement *meshElement = xmlElement->FirstChildElement("Mesh");
	while (meshElement != NULL)
	{
		Mesh *mesh = new Mesh();

		meshElement->QueryIntAttribute("id", &mesh->meshId);

		// read projection type
		str = meshElement->Attribute("type");

		if (strcmp(str, "wireframe") == 0)
		{
			mesh->type = WIREFRAME_MESH;
		}
		else
		{
			mesh->type = SOLID_MESH;
		}

		// read mesh transformations
		XMLElement *meshTransformationsElement = meshElement->FirstChildElement("Transformations");
		XMLElement *meshTransformationElement = meshTransformationsElement->FirstChildElement("Transformation");

		while (meshTransformationElement != NULL)
		{
			char transformationType;
			int transformationId;

			str = meshTransformationElement->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			mesh->transformationTypes.push_back(transformationType);
			mesh->transformationIds.push_back(transformationId);

			meshTransformationElement = meshTransformationElement->NextSiblingElement("Transformation");
		}

		mesh->numberOfTransformations = mesh->transformationIds.size();

		// read mesh faces
		char *row;
		char *cloneStr;
		int v1, v2, v3;
		XMLElement *meshFacesElement = meshElement->FirstChildElement("Faces");
		str = meshFacesElement->GetText();
		cloneStr = strdup(str);

		row = strtok(cloneStr, "\n");
		while (row != NULL)
		{
			int result = sscanf(row, "%d %d %d", &v1, &v2, &v3);

			if (result != EOF)
			{
				mesh->triangles.push_back(Triangle(v1, v2, v3));
			}
			row = strtok(NULL, "\n");
		}
		mesh->numberOfTriangles = mesh->triangles.size();
		this->meshes.push_back(mesh);

		meshElement = meshElement->NextSiblingElement("Mesh");
	}
}

void Scene::assignColorToPixel(int i, int j, Color c)
{
	this->image[i][j].r = c.r;
	this->image[i][j].g = c.g;
	this->image[i][j].b = c.b;
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	if (this->image.empty())
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			vector<Color> rowOfColors;
			vector<double> rowOfDepths;

			for (int j = 0; j < camera->verRes; j++)
			{
				rowOfColors.push_back(this->backgroundColor);
				rowOfDepths.push_back(1.01);
			}

			this->image.push_back(rowOfColors);
			this->depth.push_back(rowOfDepths);
		}
	}
	else
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			for (int j = 0; j < camera->verRes; j++)
			{
				assignColorToPixel(i, j, this->backgroundColor);
				this->depth[i][j] = 1.01;
				this->depth[i][j] = 1.01;
				this->depth[i][j] = 1.01;
			}
		}
	}
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout;

	fout.open(camera->outputFilename.c_str());

	fout << "P3" << endl;
	fout << "# " << camera->outputFilename << endl;
	fout << camera->horRes << " " << camera->verRes << endl;
	fout << "255" << endl;

	for (int j = camera->verRes - 1; j >= 0; j--)
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
		}
		fout << endl;
	}
	fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
*/
void Scene::convertPPMToPNG(string ppmFileName)
{
	string command;

	// TODO: Change implementation if necessary.
	command = "./magick convert " + ppmFileName + " " + ppmFileName + ".png";
	system(command.c_str());
}

/*
	Transformations, clipping, culling, rasterization are done here.
*/

