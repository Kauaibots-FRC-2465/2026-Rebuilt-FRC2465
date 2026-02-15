package frc.robot.subsystems;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
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
                             HEIGHT = 32, 
                             CHANNELS = 4;

    private byte[][][][] aniMatrix;
    private List<DatagramPacket> packets = new LinkedList<>();

    public static final String PATH = Filesystem.getDeployDirectory().getAbsolutePath() + File.separator + "Sprite-0001.bmp";
    

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

    public void loadMatrixData(byte[][][] matrix) {
        try (DatagramSocket socket = new DatagramSocket()) {
            int superOffset = 0;
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
                        int actualX = (y % 2 == 0) ? x : x; // z(WIDTH - 1 - x);
                        // Serpentine flip
                        // Accessing your data: matrix[y][actualX] = {W, R, G, B};
                        // R
                        int sourceY = y + (superOffset * MAX_HEIGHT);
                        if (sourceY < HEIGHT) { // Safety check
                            payload[++bufferOffset] = (byte) (matrix[sourceY][actualX][1] >> 4);
                            payload[++bufferOffset] = (byte) (matrix[sourceY][actualX][2] >> 4);
                            payload[++bufferOffset] = (byte) (matrix[sourceY][actualX][3] >> 4);
                            payload[++bufferOffset] = (byte) (matrix[sourceY][actualX][0] >> 4);
                        }
                    }
                }
                DatagramPacket packet = new DatagramPacket(payload, payload.length, address, DDP_PORT);
                packets.add(packet);
                //System.out.println(packet);
                //System.out.println(payload);
                System.out.println(payload.length);
                superOffset ++;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    public void loadIcon(BufferedImage image, int offset) throws IOException, BadImageFormatException {
        if (image == null) {
            throw new BadImageFormatException("Error: Unsupported image format or file not found.");
        }

       
        int realWidth = image.getWidth();
        int width = WIDTH;
        int height = 16;
        byte[][][] buffer = new byte[height][width][4];
        for (int w = 0; w < WIDTH; ++w) {
            for (int h = 0; h < height; ++h) {
                int hexcode = image.getRGB((w + offset) % realWidth, h);
                int red   = (hexcode >> 16) & 0xFF;
                int green = (hexcode >> 8) & 0xFF;
                int blue  = (hexcode & 0xFF);
                int white = 0;// Math.min(Math.min(RED, GREEN), BLUE);
                red   -= white;
                green -= white;
                blue  -= white;
                byte[] partial = {(byte) (white & 0xFF), (byte) (red & 0xFF), (byte) (green & 0xFF), (byte) (blue & 0xFF)};
                buffer[h][w] = partial;
            }
        }

        loadMatrixData(buffer);
        image.getRGB(1, 1);
    }

    public Command loadSingletMarquee(String path) {
        return runOnce(() -> {
            try {
                this.loadMarquee(path);
            } catch (BadImageFormatException | IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        });
    }

    public Command loadSingletAnimation(String path) {
        return runOnce(() -> {
            try {
                loadAnimation(path);
            } catch (BadImageFormatException | IOException | RuntimeException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        });
    }

    public Command loadSingledImage(String path) {
        return runOnce(() -> {
            try {
                loadIcon(path, 0);
            } catch (IOException | BadImageFormatException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        });
    }

    private void loadIcon(String path, int offset) throws IOException, BadImageFormatException {
        BufferedImage image = ImageIO.read(new File(path));
        if (image == null) {
            throw new BadImageFormatException("Error: Unsupported image format or file not found.");
        }

       
        int realWidth = image.getWidth();
        int width = WIDTH;
        int height = 16;
        byte[][][] buffer = new byte[height][width][4];
        for (int w = 0; w < WIDTH; ++w) {
            for (int h = 0; h < height; ++h) {
                int hexcode = image.getRGB((w + offset) % realWidth, h);
                int red   = (hexcode >> 16) & 0xFF;
                int green = (hexcode >> 8) & 0xFF;
                int blue  = (hexcode & 0xFF);
                int white = 0;// Math.min(Math.min(RED, GREEN), BLUE);
                red   -= white;
                green -= white;
                blue  -= white;
                byte[] partial = {(byte) (white & 0xFF), (byte) (red & 0xFF), (byte) (green & 0xFF), (byte) (blue & 0xFF)};
                buffer[h][w] = partial;
            }
        }

        loadMatrixData(buffer);
        image.getRGB(1, 1);
    }

    private int frame = 0;

   
    public void loadMarquee(String path) throws BadImageFormatException, IOException {
        BufferedImage image = ImageIO.read(new File(path));
        if (image == null) {
            throw new BadImageFormatException("Error: Unsupported image format or file not found.");
        }

       
        int realWidth = image.getWidth();
        int width = WIDTH;
        int height = 16;
        byte[][][] imageData = new byte[height][realWidth][4];
        for (int w = 0; w < realWidth; ++w) {
            for (int h = 0; h < height; ++h) {
                int hexcode = image.getRGB((w) % realWidth, h);
                int red   = (hexcode >> 16) & 0xFF;
                int green = (hexcode >> 8) & 0xFF;
                int blue  = (hexcode & 0xFF);
                int white = Math.min(Math.min(red, green), blue);
                red   -= white;
                green -= white;
                blue  -= white;
                byte[] partial = {(byte) (white & 0xFF), (byte) (red & 0xFF), (byte) (green & 0xFF), (byte) (blue & 0xFF)};
                imageData[h][w] = partial;
            }
        }

        byte[][][][] tempAnim = new byte[realWidth][height][width][4];
       
        for (int offset = 0; offset < realWidth; ++offset) {
            for (int heightOffset = 0; heightOffset < height; ++heightOffset){
                for (int dualOffset = 0; dualOffset < width; ++dualOffset) {
                    tempAnim[offset][heightOffset][dualOffset][0] = imageData[heightOffset][(offset + dualOffset) % realWidth][0];
                    tempAnim[offset][heightOffset][dualOffset][1] = imageData[heightOffset][(offset + dualOffset) % realWidth][1];
                    tempAnim[offset][heightOffset][dualOffset][2] = imageData[heightOffset][(offset + dualOffset) % realWidth][2];
                    tempAnim[offset][heightOffset][dualOffset][3] = imageData[heightOffset][(offset + dualOffset) % realWidth][3];
                }
            }
        }

        aniMatrix = tempAnim;

        image.getRGB(1, 1);

        

        packets.clear();
        for (byte[][][] frame: aniMatrix) {
            loadMatrixData(frame);
        }
    }

    public void loadAnimation(String path) throws BadImageFormatException, IOException {
        BufferedImage image = ImageIO.read(new File(path));
        if (image == null) {
            throw new BadImageFormatException("Error: Unsupported image format or file not found.");
        }

       
        int realWidth = image.getWidth();
        int width = WIDTH;
        if (realWidth % width != 0) {
            throw new RuntimeException("[AT WLEDSubsystem.loadAnimation(236:13)] Not of correct width. Please load an image which is a multiple of the current device's width.");
        }
        int height = 16;
        byte[][][] imageData = new byte[height][realWidth][4];
        for (int w = 0; w < realWidth; ++w) {
            for (int h = 0; h < height; ++h) {
                int hexcode = image.getRGB((w) % realWidth, h);
                int red   = (hexcode >> 16) & 0xFF;
                int green = (hexcode >> 8) & 0xFF;
                int blue  = (hexcode & 0xFF);
                int white = 0;// Math.min(Math.min(RED, GREEN), BLUE);
                red   -= white;
                green -= white;
                blue  -= white;
                byte[] partial = {(byte) (white & 0xFF), (byte) (red & 0xFF), (byte) (green & 0xFF), (byte) (blue & 0xFF)};
                imageData[h][w] = partial;
            }
        }

        byte[][][][] tempAnim = new byte[realWidth][height][width][4];
       
        for (int offset = 0; offset < realWidth; offset += realWidth) {
            for (int heightOffset = 0; heightOffset < height; ++heightOffset){
                for (int dualOffset = 0; dualOffset < width; ++dualOffset) {
                    tempAnim[offset][heightOffset][dualOffset][0] = imageData[heightOffset][(offset + dualOffset) % realWidth][0];
                    tempAnim[offset][heightOffset][dualOffset][1] = imageData[heightOffset][(offset + dualOffset) % realWidth][1];
                    tempAnim[offset][heightOffset][dualOffset][2] = imageData[heightOffset][(offset + dualOffset) % realWidth][2];
                    tempAnim[offset][heightOffset][dualOffset][3] = imageData[heightOffset][(offset + dualOffset) % realWidth][3];
                }
            }
        }

        aniMatrix = tempAnim;

        image.getRGB(1, 1);

        packets.clear();
        for (byte[][][] frame: aniMatrix) {
            loadMatrixData(frame);
        }
    }

    boolean isFirstRun = false;
    @Override
    public void periodic() {
        System.out.println("Periodicing...");
        if(isFirstRun) {
            try {
                loadMarquee(PATH);
            } catch (BadImageFormatException | IOException e) {}
            isFirstRun = false;
        }
        frame++;
        frame = frame % packets.size();
        try {
            this.socket.send(packets.get(frame));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        System.out.println("Periodicing done.");
       
    }

    private class BadImageFormatException extends Exception {
        protected BadImageFormatException(String message){
            super(message);
        }
    }
} 
// 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17
//                    