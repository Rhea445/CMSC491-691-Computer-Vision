#include <stdio.h>
#include <stdlib.h>
#include "dc_image.h"


#define CANNY_THRESH3 45
#define CANNY_blur3   10

#define MIN(a,b)  ( (a) < (b) ? (a) : (b) )
#define MAX(a,b)  ( (a) > (b) ? (a) : (b) )
#define ABS(x)    ( (x) <= 0 ? 0-(x) : (x) )

struct Ransac{
    int p_count;
    int line_m;
    int line_c;
    int p_x0,p_y0,p_x1,p_y1;
};

void bsort(struct Ransac list[1000], int s)
{
    int i, j;
    struct Ransac temp;
    
    for (i = 0; i < s - 1; i++)
    {
        for (j = 0; j < (s - 1-i); j++)
        {
            if (list[j].p_count < list[j + 1].p_count)
            {
                temp = list[j];
                list[j] = list[j + 1];
                list[j + 1] = temp;
            }
        }
    }
}

int main()
{
	int y,x;
	int rows3, cols3, chan3;

	//-----------------
	// Read the image    [y][x][c]   y number rows3   x cols3  c 3
	//-----------------
	byte ***img3 = LoadRgb("building.png", &rows3, &cols3, &chan3);
	printf("img3 %p rows %d cols %d chan %d\n", img3, rows3, cols3, chan3);

	
	//SaveRgbPng(img3, "out/1_img3.png", rows3, cols3);
	
	//-----------------
	// Convert to gray3scale
	//-----------------
	byte **gray3 = malloc2d(rows3, cols3);
	for (y=0; y<rows3; y++){
		for (x=0; x<cols3; x++) {
			int r = img3[y][x][0];
			int g = img3[y][x][1];
			int b = img3[y][x][2];
			gray3[y][x] =  (r+g+b) / 3;
		}
	}
	
	//Savegray3Png(gray3, "out/2_gray3.png", rows3, cols3);

	//-----------------
	// Box blur3   ToDo: Gaussian blur3 is better
	//-----------------
	
	// Box blur3 is separable, so separately blur3 x and y
	int k_x=CANNY_blur3, k_y=CANNY_blur3;
	
	// blur3 in the x dimension
	byte **blurx3 = (byte**)malloc2d(rows3, cols3);
	for (y=0; y<rows3; y++) {
		for (x=0; x<cols3; x++) {
			
			// Start and end to blur3
			int minx = x-k_x/2;      // k_x/2 left of pixel
			int maxx = minx + k_x;   // k_x/2 right of pixel
			minx = MAX(minx, 0);     // keep in bounds
			maxx = MIN(maxx, cols3);
			
			// average blur3 it
			int x2;
			int total = 0;
			int count = 0;
			for (x2=minx; x2<maxx; x2++) {
				total += gray3[y][x2];    // use "gray3" as input
				count++;
			}
			blurx3[y][x] = total / count; // blur3x is output
		}
	}
	
	//Savegray3Png(blur3x, "out/3_blur3_just_x.png", rows3, cols3);
	
	// blur3 in the y dimension
	byte **blur3 = (byte**)malloc2d(rows3, cols3);
	for (y=0; y<rows3; y++) {
		for (x=0; x<cols3; x++) {
			
			// Start and end to blur3
			int miny = y-k_y/2;      // k_x/2 left of pixel
			int maxy = miny + k_y;   // k_x/2 right of pixel
			miny = MAX(miny, 0);     // keep in bounds
			maxy = MIN(maxy, rows3);
			
			// average blur3 it
			int y2;
			int total = 0;
			int count = 0;
			for (y2=miny; y2<maxy; y2++) {
				total += blurx3[y2][x];    // use blur3x as input
				count++;
			}
			blur3[y][x] = total / count;   // blur3 is output
		}
	}
	
	//Savegray3Png(blur3, "out/3_blur3.png", rows3, cols3);
	
	
	//-----------------
	// Take the "sobel3" (magnitude of derivative)
	//  (Actually we'll make up something similar)
	//-----------------
	
	byte **sobel3 = (byte**)malloc2d(rows3, cols3);
	
	for (y=0; y<rows3; y++) {
		for (x=0; x<cols3; x++) {
			int mag=0;
			
			if (y>0)      mag += ABS(blur3[y-1][x] - blur3[y][x]);
			if (x>0)      mag += ABS(blur3[y][x-1] - blur3[y][x]);
			if (y<rows3-1) mag += ABS(blur3[y+1][x] - blur3[y][x]);
			if (x<cols3-1) mag += ABS(blur3[y][x+1] - blur3[y][x]);
			
			sobel3[y][x] = 3*mag;
		}
	}
	
	//Savegray3Png(sobel3, "out/4_sobel3.png", rows3, cols3);
	
	//-----------------
	// Non-max suppression
	//-----------------
	byte **nonmax3 = malloc2d(rows3, cols3);    // note: *this* initializes to zero!
	
	for (y=1; y<rows3-1; y++)
	{
		for (x=1; x<cols3-1; x++)
		{
			// Is it a local maximum
			int is_y_max = (sobel3[y][x] > sobel3[y-1][x] && sobel3[y][x]>=sobel3[y+1][x]);
			int is_x_max = (sobel3[y][x] > sobel3[y][x-1] && sobel3[y][x]>=sobel3[y][x+1]);
			if (is_y_max || is_x_max)
				nonmax3[y][x] = sobel3[y][x];
			else
				nonmax3[y][x] = 0;
		}
	}
	
	//Savegray3Png(nonmax3, "out/5_nonmax3.png", rows3, cols3);
	
	//-----------------
	// Final Threshold
	//-----------------
	byte **edges3 = malloc2d(rows3, cols3);    // note: *this* initializes to zero!
	
	for (y=0; y<rows3; y++) {
		for (x=0; x<cols3; x++) {
			if (nonmax3[y][x] > CANNY_THRESH3)
				edges3[y][x] = 255;
			else
				edges3[y][x] = 0;
		}
	}
    
    //Savegray3Png(edges3, "out/6_edges3.png", rows3, cols3);
    
    // chan3ge to red edges3
    
    byte ***colorimg3= malloc3d(rows3, cols3, chan3);
    
    for (y=0; y<rows3;y++){
        for(x=0; x<cols3; x++){
            colorimg3[y][x][0]=edges3[y][x];
            colorimg3[y][x][1]=0;
            colorimg3[y][x][2]=0;
        }
    }
    
    //SaveRgbPng(colorimg3, "out/7_rededges3.png", rows3, cols3);
    
    //RANSAC
	
    struct Ransac array3[1000];
    
    for(int iter=0;iter<1000;){
        
        int x0=rand()%cols3;//arr[a].xval;
        int y0=rand()%rows3;//arr[a].yval;
        int x1=rand()%cols3;//arr[b].xval;
        int y1=rand()%rows3;
        
        int dx=x1-x0;
        int dy=y1-y0;
        
        if(dx!=0 & dy!=0){
            int m=dx/dy;
            int c=x0-(m*y0);
            
            
            int pc=0;
            
            
            for (y=0; y<rows3; y++) {
                for (x=0; x<cols3; x++) {
                    int z=m*y+c;
                    if((x==z) && (colorimg3[y][x][0]==255))
                        pc++;
                }
            }
            
            array3[iter].p_count=pc;
            array3[iter].line_m=m;
            array3[iter].line_c=c;
            array3[iter].p_x0=x0;
            array3[iter].p_y0=y0;
            array3[iter].p_x1=x1;
            array3[iter].p_y1=y1;
            
            iter++;
        }
    }
    
    bsort(array3,1000);
    
    //    for(int iter=0;iter<5;iter++){
    //        printf("\n For iter=%d \n m=%d \t c=%d \t count=%d p0=%d,%d p1=%d,%d\n ",iter,array3[iter].line_m,array3[iter].line_c,array3[iter].p_count,array3[iter].p_x0,array3[iter].p_y0,array3[iter].p_x1,array3[iter].p_y1);
    //    }
    
    for(int iter=0;iter<20;iter++){
        for(y=0;y<rows3;y++){
            x=(array3[iter].line_m*y)+array3[iter].line_c;
            //            printf("\n %d \t %d \n",x,y);
            if(x>0 && colorimg3[y][x][0]==255){
                //               printf("\n %d \t %d \n",x,y);
                colorimg3[y][x][1]=255;
            }
        }
    }
    
    SaveRgbPng(colorimg3, "out/lines_building.png", rows3, cols3);
    printf("Done!\n");


    return 0;
}

/*

	printf("load image\n");
	byte *data = stbi_load("puppy.jpg", &cols3, &rows3, &chan3, 4);

	printf("data = %p\n", data);
	int rt=stbi_write_png("output.png", cols3, rows3, 4, data, cols3*4);
*/
