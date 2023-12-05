import argparse

import cv2

def main(video1_path, video2_path, output_path):
    # Open the videos
    video1 = cv2.VideoCapture(video1_path)
    video2 = cv2.VideoCapture(video2_path)

    # Get video properties (frame width, height)
    width = int(video1.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video1.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(video1.get(cv2.CAP_PROP_FPS))
    frame_count = int(video1.get(cv2.CAP_PROP_FRAME_COUNT))

    # Create a new video writer object
    combined_video = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*'XVID'), fps, (2 * width, height))

    # Process each frame
    for i in range(frame_count):
        ret1, frame1 = video1.read()
        ret2, frame2 = video2.read()

        if ret1 and ret2:
            # Resize frames if they are not the same size
            frame2_resized = cv2.resize(frame2, (width, height))

            # Combine frames horizontally
            combined_frame = cv2.hconcat([frame1, frame2_resized])

            # Write to the new video
            combined_video.write(combined_frame)
        else:
            break

    # Release video objects and writer
    video1.release()
    video2.release()
    combined_video.release()

    # Close all OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('video1_path', help='Path to the first video')
    parser.add_argument('video2_path', help='Path to the second video')
    parser.add_argument('-o', '--output', help='Path for the output video')

    args = parser.parse_args()
    video1_path = args.video1_path
    video2_path = args.video2_path
    output_path = args.output or 'combined_video.avi'

    main(video1_path, video2_path, output_path)
