#include "Regionizer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

MAKE_MODULE(Regionizer, GroundTruth)

void Regionizer::update(Regions& regions)
{
    regionSearcher(theMovementImage, theImage, regions);
}

void Regionizer::dirProvider(int &iLast, int &jLast, int &i, int &j, cv::Mat theMovementImage, bool &breakLoop, int &iMin, int &jMin, int &iMax, int &jMax)
{
    theMovementImage.convertTo(theMovementImage, CV_8UC1);
    int iDir = i - iLast;
    int jDir = j - jLast;
    int changeDir;
    int iNewChange, jNewChange, iNewNoChange, jNewNoChange;
    bool change;
    //std::cout << "A vers...: " << (theMovementImage.at<u_int8_t>(i,j) ? 1.f : 0.f)  << std::endl;
    //std::cout << theMovementImage << std::endl;
    if (theMovementImage.at<u_int8_t>(i,j) == 255)
    {
        //std::cout << "Buena" << std::endl;
        if (i != 0 || j!= 0)
        {
            //std::cout << "iDir: " << iDir << std::endl;
            //std::cout << "jDir: " << jDir << std::endl;
            if (iDir == 0 && jDir == step)
            {
                change = (theMovementImage.at<u_int8_t>(i,j) != theMovementImage.at<u_int8_t>(i-step,j));
                iNewChange = i;
                jNewChange = j+step;
                iNewNoChange = i-step;
                jNewNoChange = j;
            }
            else if (iDir == -step && jDir == 0)
            {
                change = (theMovementImage.at<u_int8_t>(i,j) != theMovementImage.at<u_int8_t>(i,j-step));
                iNewChange = i-step;
                jNewChange = j;
                iNewNoChange = i;
                jNewNoChange = j-step;
            }
            else if (iDir == 0 && jDir == -step)
            {
                change = (theMovementImage.at<u_int8_t>(i,j) != theMovementImage.at<u_int8_t>(i+step,j));
                iNewChange = i;
                jNewChange = j-step;
                iNewNoChange = i+step;
                jNewNoChange = j;
            }
            else if (iDir == step && jDir == 0)
            {
                change = (theMovementImage.at<u_int8_t>(i,j) != theMovementImage.at<u_int8_t>(i,j+step));
                iNewChange = i+step;
                jNewChange = j;
                iNewNoChange = i;
                jNewNoChange = j+step;
            }
            else
            {
                std::cout << "Algo mal..." << std::endl;
            }
            iLast = i;
            jLast = j;
            if (change)
            {
                i = iNewChange;
                j = jNewChange;
            }
            else
            {
                i = iNewNoChange;
                j = jNewNoChange;
            }
            //std::cout << "i: " << i << std::endl;
        }
        else
        {
            breakLoop = 1;
            std::cout << "breakLoop" << std::endl;
        }
        if (i < iMin)
        {
            iMin = i;
        }
        if (j < jMin)
        {
            jMin = j;
        }
        if (i > iMax)
        {
            iMax = i;
        }
        if (j > jMax)
        {
            jMax = j;
        }
    }
    else
    {
        changeDir = i;
        i = iLast;
        iLast = changeDir;
        changeDir = j;
        j = jLast;
        jLast = changeDir;
    }
    //std::cout << "iMin: " << iMin << std::endl;
}

void Regionizer::convolucionBinaria(cv::Mat imageGrey, cv::Mat mask, cv::Mat &outputImage, float threshold)
{
    int rows = imageGrey.rows;
    int cols = imageGrey.cols;
    outputImage = cv::Mat::zeros(rows, cols, CV_32F); // Imagen de salida es reemplazada por ceros
    int kRows = mask.rows;
    int kCols = mask.cols;
    int kCenterX = kCols / 2; // Centro máscara en x
    int kCenterY = kRows / 2; // Centro máscara en y
    int mm;
    int nn;
    int ii;
    int jj;
    for(int /*i=kCenterY*/i=step; i < rows-kCenterY; /*i++*/i+=step) // Recorrer filas de la imagen original
    {
        for(int /*j=kCenterX*/ j=2*step; j < cols-kCenterX; /*j++*/j+=step) // Recorrer columnas de la imagen original
        {
            for(int m=0; m < kRows; m++) // Recorrer filas de la máscara
            {
                mm = kRows - 1 - m; // Coordenada fila máscara
                for(int n=0; n < kCols; n++) // Recorrer columnas de la máscara
                {
                    nn = kCols - 1 - n; // Coordenada columna máscara
                    ii = i + (m - kCenterY); // Coordenadas filas imagen a recorrer
                    jj = j + (n - kCenterX); // Coordenadas columnas imagen a recorrer
                    if( ii >= 0 && ii < rows && jj >= 0 && jj < cols )
                    {
                        // Se genera la convolución
                        outputImage.at<float>(i,j) += imageGrey.at<float>(ii,jj) * mask.at<float>(mm,nn);
                    }
                }
            }
            // Binarizar resultado
            if (outputImage.at<float>(i,j) < threshold)
            {
                outputImage.at<float>(i,j) = 0;
            }
            else
            {
                outputImage.at<float>(i,j) = 255;
            }
        }
    }
    // Quitamos los bordes negros
    outputImage = outputImage.colRange(kCenterX, cols - kCenterX);
    outputImage = outputImage.rowRange(kCenterY, rows - kCenterY);
}

void Regionizer::regionSearcher(cv::Mat theMovementImage, cv::Mat theImage, Regions &regions)
{
    // Filtrar imagen

    // Convolución
    float threshold = 57;
    cv::Mat filteredImage;
    cv::Mat input = theMovementImage.clone();
    input.convertTo(input, CV_32FC1);
    convolucionBinaria(input, mask, filteredImage, threshold);
    //imshow("Original", theMovementImage);
    //imshow("Filtrada", filteredImage);

    regions.regions.clear();

    for (int i = step+1; i < theImage.rows-step; i += step)
    {
        int iLast = i;
        for (int j = 2*step+1; j < theImage.cols-step; j += step)
        {
            float movementPixel = filteredImage.at<float>(i,j) ? 1.f : 0.f;
            //filteredImage.at<float>(i,j) = 255;
            //imshow("Filtrada", filteredImage);
            if (movementPixel == 1)
            {
                //std::cout << "Movimiento detectado.." << std::endl;
                //int depth = 0;
                ColorModel::Colors color = theColorModel.getColor(theImage.at<cv::Vec3b>(i,step));
                int iStart = i;
                int jStart = j;
                int count = 0;
                int iDir = 1;
                int jDir = -1;
                bool change = 1;
                int jLast = j-step;
                bool breakLoop = 0;
                int iMin = 1000000, jMin = 1000000, iMax = 0, jMax = 0;
                //std::cout << "i resta: " << std::abs(i - iStart) << "; j resta: " << std::abs(j - jStart) << std::endl;
                //std::cout << "s x step: " << 2*step << std::endl;
                //while (std::abs(i - iStart) < 2*step && std::abs(j - jStart) < 2*step && count < 400 || count < 10 )
                cv::Mat filteredImageCopy = filteredImage.clone();
                while ((std::abs(i-iStart) > 2*step || std::abs(j-jStart) > 2*step)  || count < 3*step )
                {
                    //std::cout << std::abs(i-iStart) << std::endl;
                    //std::cout << std::abs(j-jStart) << std::endl;
                    count++;
                    //std::cout << "count: " << count << std::endl;
                    dirProvider(iLast, jLast, i, j, filteredImage, breakLoop, iMin, jMin, iMax, jMax);
                    /*cv::Point pt;
                    pt.x = j;
                    pt.y = i;*/
                    //cv::Point pt2;
                    //pt2.x = i+3;
                    //pt2.y = j+3;
                    //cv::circle(filteredImageCopy, pt, 3, cv::Scalar( 255, 0, 0 ), 1);
                    //cv::circle(regionViewer, pt2, 3, cv::Scalar( 255, 0, 0 ), 1);
                    if (breakLoop)
                    {
                        break;
                    }
                    //cv::imshow("Movement", filteredImageCopy);

                }
                // Eliminar de la imagen lo ya detectado
                cv::Point ptSup;
                ptSup.x = jMin;
                ptSup.y = iMin;
                cv::Point ptInf;
                ptSup.x = jMax;
                ptSup.y = iMax;
                cv::rectangle(filteredImage, ptSup, ptInf, cv::Scalar(0), -1);
                //cv::imshow("Rectangle", filteredImage);
                // Volver a partir desde donde se estaba
                i = iStart;
                j = jStart;
                // Actualizar las regiones
                Vector2<int> leftUpper(jMin,iMin);
                Vector2<int> rightBottom(jMax,iMax);
                Vector2<int> center((jMax-jMin)/2,(iMax-iMin)/2);
                regions.regions.push_back(Regions::Rect(center,leftUpper,rightBottom,color));
                //std::cout << "Región recorrida." << std::endl;
                break;
            }

        }
    }
}
