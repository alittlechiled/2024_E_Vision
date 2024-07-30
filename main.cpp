#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <map>
using namespace cv;
using namespace std;

struct board_block{
    int is_Chequer_exist(vector<RotatedRect> &white,vector<RotatedRect> &black);
    double norm(board_block &other);
    cv::Point2f point_in_center;
    cv::Mat black_img;
    cv::Mat white_img;
    int order;
    bool is_exist_Chequer;
    cv::RotatedRect rect;
};

int board_block::is_Chequer_exist(vector<RotatedRect> &white,vector<RotatedRect> &black){
    cv::Point2f point[4];
    this->rect.points(point);
    int b_c = 0,w_c = 0;
    std::vector<cv::Point2f> rectangle = {point[0],point[1],point[2],point[3]};
    for(auto it:white){
        double result = cv::pointPolygonTest(rectangle, it.center, false);
        w_c +=  (result > 0 );
    }

    for(auto it:black){
        double result = cv::pointPolygonTest(rectangle, it.center, false);
        b_c +=  (result > 0 );
    }

    int id = 0;

    if(w_c > 0)
        id = 1;

    if(w_c == 0 && b_c > 0)
        id = -1; 
    return id;
}

struct board{
    vector<int> get_station(vector<RotatedRect> &white,vector<RotatedRect> &black);
    double get_angle();
    cv::Mat get_board_transform();
    void update_board_block(map<int,board_block> &new_boards);
    int chequers[3][3];
    map<int,board_block> boards;
    double target_angle;
};

double board::get_angle(){
    std::vector<cv::Point2f> direction_vector;
    double angle = 0;
    for(int i = 1;i < 4;++i){
        direction_vector.push_back(boards[i].rect.center - boards[i+6].rect.center);
    }

    for(auto it:direction_vector){
        angle += atan2(it.y,it.x)*180/M_PI;
    }

    return angle/3 + 90;
}

vector<int> board::get_station(vector<RotatedRect> &white,vector<RotatedRect> &black){
    vector<int> resualt;
    for(int i = 0;i<3;++i){
        for(int j = 0;j<3;++j){
            chequers[i][j] = boards[i*3 + j + 1].is_Chequer_exist(white,black);
            resualt.push_back(chequers[i][j]);
        }
    }
    return resualt;
}

int main() {
    board Board_;
    int board_threshold = 180;
    int white_threshold = 100;
    int black_threshold = 170;

    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cout << "Cannot open the camera" << endl;
        return -1;
    }

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) {
            cout << "Captured frame is empty" << endl;
            break;
        }

        Size newSize(frame.cols, frame.rows);
        Mat frame_resized;
        resize(frame, frame_resized, newSize);

        vector<Mat> channels;
        split(frame_resized, channels);

        int threshold_value = 100;
        cv::Mat board = channels[2] + channels[1] - channels[0],black_img,whiht_img;
        cv::imshow("board",board);

        threshold(board, black_img, board_threshold, 255, THRESH_BINARY);
        threshold(board, board, board_threshold, 255, THRESH_BINARY);
        
        cv::imshow("b",channels[0]);

        threshold(channels[0],whiht_img, white_threshold, 255, THRESH_BINARY);


        cv::imshow("board",board);

        Mat struct1,struct2,struct3;
        struct1=getStructuringElement(0,Size(34,34));
        struct2=getStructuringElement(0,Size(3,3));
        struct3=getStructuringElement(0,Size(20,20));

        cv::dilate( black_img, black_img, struct1 );
        cv::imshow("black_img1",black_img);
        erode( black_img, black_img, struct3 );
        erode( whiht_img, whiht_img, struct2 );

        cv::imshow("whiht_img",whiht_img);
        cv::imshow("black_img",black_img);

        vector<vector<Point>> contours,white_contours,black_contours;  
        vector<Vec4i> hierarchy;

        findContours(board,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());
        findContours(whiht_img,white_contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());
        findContours(black_img,black_contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());

        vector<RotatedRect> rects; 
        vector<RotatedRect> board_rects,black_rects,white_rects;
        vector<map<int,cv::Rect>> boards;

        vector<float> radius;
        for(auto it:white_contours){
            if(it.size()<20) continue;
            float r;
            cv::Point2f c_c;
            auto temp = cv::minAreaRect(it);
            cv::minEnclosingCircle(it,c_c,r);
            if((temp.size.area() > 6000 ||  temp.size.area() < 2000) ) continue;
                white_rects.push_back(temp);
        }

        for(auto it:black_contours){
            if(it.size()<20) continue;
            float r;
            cv::Point2f c_c;
            auto temp = cv::minAreaRect(it);
            cv::minEnclosingCircle(it,c_c,r);
            if((temp.size.area() > 4000 ||  temp.size.area() < 1000)) continue;
            black_rects.push_back(temp);
        }

        for(auto it:contours){
            if(it.size()<10) continue;
            float r;
            cv::Point2f c_c;
            rects.push_back(cv::minAreaRect(it));
            auto temp = cv::minAreaRect(it);
            cv::minEnclosingCircle(it,c_c,r);
            radius.push_back(r);            
            if(temp.size.area() < 4000  || temp.size.area()/(r*r*M_PI) >= 1) continue;
                board_rects.push_back(temp);

        }
        
        if(board_rects.size() != 9) continue;

        std::vector<cv::Point2f> board_points;
        std::vector<cv::Point2f> board_points_in_center;
        cv::Point2f board_center = cv::Point2f(0,0);
        double distance = 0;
        for(auto it:board_rects){
            board_center = board_center + it.center;
            board_points.push_back(it.center);
        }
        
        board_center = board_center / 9;
        board_center.y = -board_center.y;

        for(auto it:board_points){
            it.y = -it.y;
            board_points_in_center.push_back(it - board_center);

            distance += cv::norm(it - board_center);
        }
        distance/=45;
        for(int i = 0; i < int(board_points_in_center.size()); i++){
            cv::Point2f points[4]; 
            board_rects[i].points(points);
            auto one_center = board_rects[i].center;
            one_center.y = -one_center.y;
            one_center = (one_center - board_center);

            double angle = atan2(one_center.y,one_center.x)*180/M_PI;

            one_center = one_center/distance;

            int distance_center = 2*cv::norm(cv::Point2i(int(one_center.x),int(one_center.y)));

            int x =  int(one_center.x),y = int(one_center.y);
            string r  = to_string(distance_center);
            
            int number;
            if(distance_center == 0)
                number = 5;
            else{
                if(distance_center >= 9){
                    if(angle >= 90 && angle <= 180)
                        number = 1;
                    else if(angle > 0 && angle < 90)
                        number = 3;
                    else if(angle > -180 && angle < -90)
                        number = 7;
                    else
                        number = 9;
                }else{
                    if(angle >= 45 && angle <= 135)
                        number = 2;
                    else if(angle > 135 || angle < -135)
                        number = 4;
                    else if(angle < 45 && angle > -45)
                        number = 6;
                    else{
                        number = 8;
                    }       
                }
            }

            board_block b_b_;

            int pixel = 0;

            b_b_.order = number;
            b_b_.rect = board_rects[i];
            b_b_.point_in_center = board_points_in_center[i];

            cv::Point2f vertex[4];
            b_b_.rect.points(vertex);

            pixel += black_img.at<uchar>(int(b_b_.rect.center.x),int(b_b_.rect.center.y));
        
            Board_.boards[number] = b_b_;

            cv::putText(frame, to_string(number), board_rects[i].center, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2, 8, 0);
        }

        auto resualt = Board_.get_station(white_rects,black_rects);
        
        for(int i = 0;i<3;++i){
            for(int j = 0;j<3;++j){
                std::cout<<resualt[i*3 + j]<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout<<"-------------------------------------------"<<std::endl;

        auto angle = Board_.get_angle();
        cv::putText(frame,"angle:" + to_string(angle), cv::Point2f(50,50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2, 8, 0);
        for(auto it:black_rects){
            cv::Point2f vertex[4];
	        it.points(vertex);

            for (int i = 0; i < 4; i++)
            {
                cv::line(frame, vertex[i], vertex[(i + 1) % 4], cv::Scalar(255, 0, 0),2);
            }

        }

        for(auto it:white_rects){
            cv::Point2f vertex[4];
	        it.points(vertex);

            for (int i = 0; i < 4; i++)
            {
                cv::line(frame, vertex[i], vertex[(i + 1) % 4], cv::Scalar(0, 0, 255),2);
            }

        }

        cv::imshow("frame",frame);

        if (waitKey(30) >= 0) break;
    }

    cap.release();
    destroyAllWindows();

    return 0;
}
