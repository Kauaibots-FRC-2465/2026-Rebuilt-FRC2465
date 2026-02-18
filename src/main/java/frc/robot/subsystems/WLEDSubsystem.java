package frc.robot.subsystems;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.LinkedList;
import java.util.List;

import javax.imageio.ImageIO;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

public class WLEDSubsystem implements Subsystem {
    DatagramSocket socket;

    private static final String IP_ADDRESS = "10.24.65.13";
    private static final int DDP_PORT = 4048,  
                             WIDTH = 32, 
                             MAX_HEIGHT = 8, 
                             HEIGHT = 16, 
                             CHANNELS = 4;

    private byte[][][][] aniMatrix;
    private List<DatagramPacket> packets = new LinkedList<>();

    private int frame = 0;


    public static final String PATH_1 = Filesystem.getDeployDirectory().getAbsolutePath() + File.separator + "Sprite-0001.bmp";
    public static final String PATH_2 = Filesystem.getDeployDirectory().getAbsolutePath() + File.separator + "Sprite-0002.bmp";
    

    public WLEDSubsystem() {
        System.out.println("Initializing...");
        CommandScheduler.getInstance().registerSubsystem(this);
        try {
            socket = new DatagramSocket();
        } catch (SocketException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
    
    private boolean isFirstRun = true;
    @Override
    public void periodic() {
        if(isFirstRun) {
            try {
                packets = loadMarquee(PATH_1);
                System.out.println("Loaded image");
            } catch (BadImageFormatException | IOException e) {
                System.out.println("Unable to load");
            }
            isFirstRun = false;
        }
        if (packets.size() > 0) {
            frame++;
            frame = frame % packets.size();
            try {
                this.socket.send(packets.get(frame));
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

    public Command loadSingletMarquee(String path) {
        List<DatagramPacket> datagramPackets = new LinkedList<>();
        try {
            datagramPackets = loadMarquee(path);
        } catch (BadImageFormatException | IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        final List<DatagramPacket> finalPackets = datagramPackets;
        return runOnce(() -> {
            packets = finalPackets;
            System.out.println("Loaded singlet marquee");
        });
    }

    public Command loadSingletAnimation(String path) {
        List<DatagramPacket> datagramPackets = new LinkedList<>();
        try {
            datagramPackets = loadAnimation(path);
        } catch (BadImageFormatException | IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        final List<DatagramPacket> finalPackets = datagramPackets;
        return runOnce(() -> {
            packets = finalPackets;
            System.out.println("Loaded singlet animation");
        });
    }

    public Command loadSingletImage(String path) {
        List<DatagramPacket> datagramPackets = new LinkedList<>();
        try {
            datagramPackets = loadIcon(path);
        } catch (IOException | BadImageFormatException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        final List<DatagramPacket> finalPackets = datagramPackets;
        return runOnce(() -> {
            packets = finalPackets;
            System.out.println("Loaded singlet image");
        });
    }

    private List<DatagramPacket> loadMarquee(String path) throws BadImageFormatException, IOException {
        BufferedImage image = ImageIO.read(new File(path));
        if (image == null) {
            throw new BadImageFormatException("Error: Unsupported image format or file not found.");
        }

       
        int realWidth = image.getWidth();
        int width = WIDTH;
        int height = 16;
        byte[][][] imageData = getImageData(image);
        /*
        byte[][][][] tempAnim = new byte[realWidth][height][width][4];
       
        for (int widthOffset = 0; widthOffset < realWidth; ++widthOffset) {
            for (int heightOffset = 0; heightOffset < height; ++heightOffset) {
                for (int dualOffset = 0; dualOffset < width; ++dualOffset) {
                    //System.out.println(widthOffset);
                    //System.out.println(heightOffset);
                    //System.out.println(dualOffset);
                    //System.out.println(realWidth);
                    tempAnim[widthOffset][heightOffset][dualOffset][0] = imageData[heightOffset][(widthOffset + dualOffset) % realWidth][0];
                    tempAnim[widthOffset][heightOffset][dualOffset][1] = imageData[heightOffset][(widthOffset + dualOffset) % realWidth][1];
                    tempAnim[widthOffset][heightOffset][dualOffset][2] = imageData[heightOffset][(widthOffset + dualOffset) % realWidth][2];
                    tempAnim[widthOffset][heightOffset][dualOffset][3] = imageData[heightOffset][(widthOffset + dualOffset) % realWidth][3];
                }
            }
        }

         */
        aniMatrix = getAnim(realWidth, height, width, imageData, 1);
        
        List<DatagramPacket> thesePackets = new LinkedList<>();
        for (byte[][][] frame: aniMatrix) {
            thesePackets.addAll(loadMatrixData(frame));
        }
        return thesePackets;
    }

    private List<DatagramPacket> loadAnimation(String path) throws BadImageFormatException, IOException {
        BufferedImage image = ImageIO.read(new File(path));
        if (image == null) {
            throw new BadImageFormatException("Error: Unsupported image format or file not found.");
        }

       
        int realWidth = image.getWidth();
        int width = WIDTH;
        assert realWidth % width == 0: "[AT WLEDSubsystem.loadAnimation(236:13)] Not of correct width. Please load an image which is an integer multiple of the current device's width.";
        int height = 16;
        byte[][][] imageData = getImageData(image);
        /*
        byte[][][][] tempAnim = new byte[realWidth][height][width][4];
       
        for (int widthOffset = 0; widthOffset < realWidth; widthOffset += realWidth) {
            for (int heightOffset = 0; heightOffset < height; ++heightOffset){
                for (int channelOffset = 0; channelOffset < width; ++channelOffset) {
                    tempAnim[widthOffset][heightOffset][channelOffset][0] = imageData[heightOffset][(widthOffset + channelOffset) % realWidth][0];
                    tempAnim[widthOffset][heightOffset][channelOffset][1] = imageData[heightOffset][(widthOffset + channelOffset) % realWidth][1];
                    tempAnim[widthOffset][heightOffset][channelOffset][2] = imageData[heightOffset][(widthOffset + channelOffset) % realWidth][2];
                    tempAnim[widthOffset][heightOffset][channelOffset][3] = imageData[heightOffset][(widthOffset + channelOffset) % realWidth][3];
                }
            }
        }
            
         */

        aniMatrix = getAnim(realWidth/width, height, width, imageData, realWidth);

        List<DatagramPacket> thesePackets = new LinkedList<>();
        for (byte[][][] frame: aniMatrix) {
            thesePackets.addAll(loadMatrixData(frame));
        }
        return thesePackets;
    }
    
    private List<DatagramPacket> loadMatrixData(byte[][][] matrix) throws UnknownHostException {
        int superOffset = 0;
        List<DatagramPacket> matrixData = new LinkedList<>();
        while (WIDTH * MAX_HEIGHT * superOffset < WIDTH * HEIGHT){
            InetAddress address = InetAddress.getByName(IP_ADDRESS);
            // Header: Flags (0x41), Count, Offset, etc.
            // For a 16x16 WRGB matrix, data length is 16*16*4 = 1024 bytes
            byte[] payload = new byte[10 + (WIDTH * MAX_HEIGHT * CHANNELS)];
            // DDP Header payload[0] = 0x41;
            payload[0] = 0b01000001;
            // Flags: Version 1, Push enabled
            payload[1] = 0b00000001;
            // Sequence (optional)
            payload[2] = 0b00011011;
            // Data Type: RGB or WRGB (Depending on WLED config)
            payload[3] = 0b00000001;
            // Destination ID
            // Offset (4-7) and Length (8-9) - simplified for small matrices
            int offset = WIDTH * MAX_HEIGHT * CHANNELS * superOffset;
            payload[4] = (byte) ((offset >> 24) & 0xFF); // High byte
            payload[5] = (byte) ((offset >> 16) & 0xFF);        // Low byte
            payload[6] = (byte) ((offset >> 8) & 0xFF); // High byte
            payload[7] = (byte) ((offset >> 0) & 0xFF);        // Low byte
            int dataLength = WIDTH * MAX_HEIGHT * CHANNELS;
            payload[8] = (byte) ((dataLength >> 8) & 0xFF); // High byte
            payload[9] = (byte) (dataLength & 0xFF);        // Low byte
            int bufferOffset = 9;
            for (int y = 0; y < MAX_HEIGHT; y++) {
                for (int x = 0; x < WIDTH; x++) {
                    // Accessing your data: matrix[y][x] = {W, R, G, B};
                    int sourceY = y + (superOffset * MAX_HEIGHT);
                    if (sourceY < HEIGHT && x < WIDTH) { // Safety check
                        //System.out.println(bufferOffset + "," + sourceY + ", " + x);
                        payload[++bufferOffset] = (byte) (matrix[sourceY][x][1]);
                        payload[++bufferOffset] = (byte) (matrix[sourceY][x][2]);
                        payload[++bufferOffset] = (byte) (matrix[sourceY][x][3]);
                        payload[++bufferOffset] = (byte) (matrix[sourceY][x][0]);
                    }
                }
            }
            DatagramPacket packet = new DatagramPacket(payload, payload.length, address, DDP_PORT);
            //System.out.println(packet);
            //System.out.println(payload);
            //System.out.println(payload.length);
            matrixData.add(packet);
            superOffset ++;
        }
        return matrixData;
    }

    
    private byte[][][] getImageData(BufferedImage image) {
        int realWidth = image.getWidth();
        System.out.println(realWidth);
        int height = 16;
        byte[][][] buffer = new byte[height][realWidth][4];

        for (int w = 0; w < realWidth; ++w) {
            for (int h = 0; h < height; ++h) {
                int hexcode = image.getRGB(w % realWidth, h);
                int red   = (hexcode >> 16) & 0xFF;
                int green = (hexcode >> 8) & 0xFF;
                int blue  = (hexcode & 0xFF);
                int white = Math.min(Math.min(red, green), blue);
        
                red   -= white;
                green -= white;
                blue  -= white;

                byte[] partial = {(byte) (white & 0xFF), (byte) (red & 0xFF), (byte) (green & 0xFF), (byte) (blue & 0xFF)};
                
                buffer[h][w] = partial;
            }
        }
        return buffer;
    }

    
    private byte[][][] getClippedImageData(BufferedImage image, int clipLength) {
        int realWidth = image.getWidth();
        System.out.println(realWidth);
        int height = 16;
        byte[][][] buffer = new byte[height][realWidth][4];

        for (int w = 0; w < clipLength; ++w) {
            for (int h = 0; h < height; ++h) {
                int hexcode = image.getRGB(w % realWidth, h);
                int red   = (hexcode >> 16) & 0xFF;
                int green = (hexcode >> 8) & 0xFF;
                int blue  = (hexcode & 0xFF);
                int white = Math.min(Math.min(red, green), blue);
        
                red   -= white;
                green -= white;
                blue  -= white;

                byte[] partial = {(byte) (white & 0xFF), (byte) (red & 0xFF), (byte) (green & 0xFF), (byte) (blue & 0xFF)};
                
                buffer[h][w] = partial;
            }
        }
        return buffer;
    }

    
    private List<DatagramPacket> loadIcon(String path) throws IOException, BadImageFormatException {
        BufferedImage image = ImageIO.read(new File(path));
        if (image == null) {
            throw new BadImageFormatException("Error: Unsupported image format or file not found.");
        }
        byte[][][] buffer = getClippedImageData(image, WIDTH);
        return loadMatrixData(buffer);
    }

    /*
     * Calculates the animation based on imageData and stride
     * @return A byte[][][][] of form byte[frames][height][width][channels]
     */
    private byte[][][][] getAnim(int frames, int height, int width, byte[][][] imageData, int stride) {
        byte[][][][] tempAnim = new byte[frames][height][width][4];
       
        for (int widthOffset = 0; widthOffset < frames; widthOffset += stride) {
            for (int heightOffset = 0; heightOffset < height; ++heightOffset){
                for (int channelOffset = 0; channelOffset < width; ++channelOffset) {
                    tempAnim[widthOffset][heightOffset][channelOffset][0] = imageData[heightOffset][(widthOffset + channelOffset) % frames][0];
                    tempAnim[widthOffset][heightOffset][channelOffset][1] = imageData[heightOffset][(widthOffset + channelOffset) % frames][1];
                    tempAnim[widthOffset][heightOffset][channelOffset][2] = imageData[heightOffset][(widthOffset + channelOffset) % frames][2];
                    tempAnim[widthOffset][heightOffset][channelOffset][3] = imageData[heightOffset][(widthOffset + channelOffset) % frames][3];
                }
            }
        }
        return tempAnim;
    }
   
    private class BadImageFormatException extends Exception {
        protected BadImageFormatException(String message){
            super(message);
        }
    }


} 
// 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17
//                    