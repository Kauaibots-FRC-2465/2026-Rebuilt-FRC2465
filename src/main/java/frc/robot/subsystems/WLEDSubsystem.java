package frc.robot.subsystems;

import java.awt.AlphaComposite;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Iterator;

import javax.imageio.ImageIO;
import javax.imageio.ImageReader;
import javax.imageio.metadata.IIOMetadata;
import javax.imageio.metadata.IIOMetadataNode;
import javax.imageio.stream.ImageInputStream;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class WLEDSubsystem implements Subsystem {
    private static final String IP_ADDRESS = "10.24.65.13";
    private static final int DDP_PORT = 4048;
    private static final int WIDTH = 32;
    private static final int MAX_HEIGHT = 8;
    private static final int HEIGHT = 16;
    private static final int CHANNELS = 4;

    private final DatagramSocket socket;

    private ActivePlayback activePlayback;

    public static final class PreparedPlaybackPackets {
        private final DatagramPacket[] packets;
        private final int packetsPerFrame;
        private final double[] frameDisplayDurationsSeconds;

        private PreparedPlaybackPackets(
                DatagramPacket[] packets,
                int packetsPerFrame,
                double[] frameDisplayDurationsSeconds) {
            this.packets = packets;
            this.packetsPerFrame = packetsPerFrame;
            this.frameDisplayDurationsSeconds = frameDisplayDurationsSeconds;
        }

        public int getPacketCount() {
            return packets.length;
        }

        public int getPacketsPerFrame() {
            return packetsPerFrame;
        }

        public int getFrameCount() {
            if (packetsPerFrame <= 0) {
                return 0;
            }
            return packets.length / packetsPerFrame;
        }

        public double getFrameDisplayDurationSeconds(int frameIndex) {
            return frameDisplayDurationsSeconds[frameIndex];
        }

        private boolean isEmpty() {
            return packets.length == 0
                    || packetsPerFrame <= 0
                    || frameDisplayDurationsSeconds.length == 0;
        }
    }

    private static final class ActivePlayback {
        private final PreparedPlaybackPackets playback;
        private final Timer frameTimer = new Timer();
        private int currentFrameIndex;

        private ActivePlayback(PreparedPlaybackPackets playback) {
            this.playback = playback;
            this.currentFrameIndex = 0;
            this.frameTimer.start();
            this.frameTimer.reset();
        }
    }

    private static final class GifFrameMetadata {
        private final int left;
        private final int top;
        private final int width;
        private final int height;
        private final String disposalMethod;
        private final double displayDurationSeconds;

        private GifFrameMetadata(
                int left,
                int top,
                int width,
                int height,
                String disposalMethod,
                double displayDurationSeconds) {
            this.left = left;
            this.top = top;
            this.width = width;
            this.height = height;
            this.disposalMethod = disposalMethod;
            this.displayDurationSeconds = displayDurationSeconds;
        }
    }

    private static final class FrameData {
        private final byte[][][] pixels;
        private final double displayDurationSeconds;

        private FrameData(byte[][][] pixels, double displayDurationSeconds) {
            this.pixels = pixels;
            this.displayDurationSeconds = displayDurationSeconds;
        }
    }

    private static final class GifCompositeState {
        private final BufferedImage canvas = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_INT_ARGB);
        private GifFrameMetadata previousFrameMetadata;
        private BufferedImage restoreToPreviousSnapshot;
    }

    public WLEDSubsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);

        DatagramSocket createdSocket = null;
        try {
            createdSocket = new DatagramSocket();
        } catch (SocketException e) {
            DriverStation.reportError("WLED socket initialization failed: " + e.getMessage(), false);
        }
        socket = createdSocket;
        activePlayback = null;
    }

    @Override
    public void periodic() {
        if (socket == null || activePlayback == null || activePlayback.playback.isEmpty()) {
            return;
        }

        int packetsPerFrame = activePlayback.playback.getPacketsPerFrame();
        int frameStartPacketIndex = activePlayback.currentFrameIndex * packetsPerFrame;
        for (int packetOffset = 0; packetOffset < packetsPerFrame; packetOffset++) {
            DatagramPacket packet = activePlayback.playback.packets[frameStartPacketIndex + packetOffset];
            try {
                socket.send(packet);
            } catch (IOException e) {
                DriverStation.reportError("WLED packet send failed: " + e.getMessage(), false);
                return;
            }
        }

        double frameDurationSeconds =
                activePlayback.playback.getFrameDisplayDurationSeconds(activePlayback.currentFrameIndex);
        if (frameDurationSeconds <= 0.0 || activePlayback.frameTimer.hasElapsed(frameDurationSeconds)) {
            activePlayback.currentFrameIndex++;
            if (activePlayback.currentFrameIndex >= activePlayback.playback.getFrameCount()) {
                activePlayback.currentFrameIndex = 0;
            }
            activePlayback.frameTimer.restart();
        }
    }

    public PreparedPlaybackPackets prepareMarquee(String path) {
        try {
            BufferedImage image = readImage(path);
            byte[][][] imageData = getImageData(image);
            int frameCount = image.getWidth();
            return buildPlaybackPackets(
                    frameCount,
                    frameIndex -> new FrameData(extractWrappedFrame(imageData, frameIndex), 0.0));
        } catch (BadImageFormatException | IOException e) {
            return reportPreparationFailure("marquee", path, e);
        }
    }

    public PreparedPlaybackPackets prepareHorizontalAnimationStrip(String path) {
        try {
            BufferedImage image = readImage(path);
            int realWidth = image.getWidth();
            if (realWidth % WIDTH != 0) {
                throw new BadImageFormatException(
                        "Animation width must be an integer multiple of " + WIDTH + " pixels.");
            }
            byte[][][] imageData = getImageData(image);
            int frameCount = realWidth / WIDTH;
            return buildPlaybackPackets(
                    frameCount,
                    frameIndex -> new FrameData(extractClippedFrame(imageData, frameIndex * WIDTH), 0.0));
        } catch (BadImageFormatException | IOException e) {
            return reportPreparationFailure("animation", path, e);
        }
    }

    public PreparedPlaybackPackets prepareImage(String path) {
        try {
            BufferedImage image = readImage(path);
            byte[][][] imageData = getImageData(image);
            DatagramPacket[] packets = buildPacketsForFrame(extractClippedFrame(imageData, 0));
            return new PreparedPlaybackPackets(packets, packets.length, new double[] {0.0});
        } catch (BadImageFormatException | IOException e) {
            return reportPreparationFailure("image", path, e);
        }
    }

    public PreparedPlaybackPackets prepareGIF(String path) {
        File gifFile = new File(path);
        try (ImageInputStream imageInputStream = ImageIO.createImageInputStream(gifFile)) {
            if (imageInputStream == null) {
                throw new BadImageFormatException("GIF file could not be opened.");
            }

            Iterator<ImageReader> imageReaders = ImageIO.getImageReaders(imageInputStream);
            if (!imageReaders.hasNext()) {
                throw new BadImageFormatException("No GIF reader is available for this file.");
            }

            ImageReader imageReader = imageReaders.next();
            try {
                imageReader.setInput(imageInputStream, false, false);
                validateGifCanvasDimensions(imageReader);
                int frameCount = imageReader.getNumImages(true);
                if (frameCount <= 0) {
                    throw new BadImageFormatException("GIF contains no frames.");
                }

                GifCompositeState compositeState = new GifCompositeState();
                return buildPlaybackPackets(
                        frameCount,
                        frameIndex -> buildCompositedGifFrame(imageReader, frameIndex, compositeState));
            } finally {
                imageReader.dispose();
            }
        } catch (BadImageFormatException | IOException e) {
            return reportPreparationFailure("gif", path, e);
        }
    }

    public Command showMarquee(String path) {
        PreparedPlaybackPackets playbackPackets = prepareMarquee(path);
        return runOnce(() -> setActivePlayback(playbackPackets)).ignoringDisable(true);
    }

    public Command showHorizontalAnimationStrip(String path) {
        PreparedPlaybackPackets playbackPackets = prepareHorizontalAnimationStrip(path);
        return runOnce(() -> setActivePlayback(playbackPackets)).ignoringDisable(true);
    }

    public Command showImage(String path) {
        PreparedPlaybackPackets playbackPackets = prepareImage(path);
        return runOnce(() -> setActivePlayback(playbackPackets)).ignoringDisable(true);
    }

    public Command showGIF(String path) {
        PreparedPlaybackPackets playbackPackets = prepareGIF(path);
        return runOnce(() -> setActivePlayback(playbackPackets)).ignoringDisable(true);
    }

    public void setActivePlayback(PreparedPlaybackPackets playbackPackets) {
        if (playbackPackets == null || playbackPackets.isEmpty()) {
            activePlayback = null;
            return;
        }

        activePlayback = new ActivePlayback(playbackPackets);
    }

    private PreparedPlaybackPackets buildPlaybackPackets(int frameCount, FrameBuilder frameBuilder)
            throws IOException, BadImageFormatException {
        if (frameCount <= 0) {
            return new PreparedPlaybackPackets(new DatagramPacket[0], 0, new double[0]);
        }

        FrameData firstFrameData = frameBuilder.buildFrame(0);
        DatagramPacket[] firstFramePackets = buildPacketsForFrame(firstFrameData.pixels);
        int packetsPerFrame = firstFramePackets.length;
        DatagramPacket[] packets = new DatagramPacket[frameCount * packetsPerFrame];
        double[] frameDisplayDurationsSeconds = new double[frameCount];
        System.arraycopy(firstFramePackets, 0, packets, 0, packetsPerFrame);
        frameDisplayDurationsSeconds[0] = firstFrameData.displayDurationSeconds;

        int packetIndex = packetsPerFrame;
        for (int frameIndex = 1; frameIndex < frameCount; frameIndex++) {
            FrameData frameData = frameBuilder.buildFrame(frameIndex);
            DatagramPacket[] framePackets = buildPacketsForFrame(frameData.pixels);
            System.arraycopy(framePackets, 0, packets, packetIndex, packetsPerFrame);
            frameDisplayDurationsSeconds[frameIndex] = frameData.displayDurationSeconds;
            packetIndex += packetsPerFrame;
        }

        return new PreparedPlaybackPackets(packets, packetsPerFrame, frameDisplayDurationsSeconds);
    }

    private DatagramPacket[] buildPacketsForFrame(byte[][][] matrix) throws UnknownHostException {
        int packetsPerFrame = (matrix.length + MAX_HEIGHT - 1) / MAX_HEIGHT;
        DatagramPacket[] packets = new DatagramPacket[packetsPerFrame];
        InetAddress address = InetAddress.getByName(IP_ADDRESS);

        for (int packetIndex = 0; packetIndex < packetsPerFrame; packetIndex++) {
            byte[] payload = new byte[10 + (WIDTH * MAX_HEIGHT * CHANNELS)];
            payload[0] = 0b01000001;
            payload[1] = 0b00000001;
            payload[2] = 0b00011011;
            payload[3] = 0b00000001;

            int offset = WIDTH * MAX_HEIGHT * CHANNELS * packetIndex;
            payload[4] = (byte) ((offset >> 24) & 0xFF);
            payload[5] = (byte) ((offset >> 16) & 0xFF);
            payload[6] = (byte) ((offset >> 8) & 0xFF);
            payload[7] = (byte) (offset & 0xFF);

            int dataLength = WIDTH * MAX_HEIGHT * CHANNELS;
            payload[8] = (byte) ((dataLength >> 8) & 0xFF);
            payload[9] = (byte) (dataLength & 0xFF);

            int bufferOffset = 9;
            for (int y = 0; y < MAX_HEIGHT; y++) {
                for (int x = 0; x < WIDTH; x++) {
                    int sourceY = y + (packetIndex * MAX_HEIGHT);
                    if (sourceY < matrix.length) {
                        payload[++bufferOffset] = matrix[sourceY][x][1];
                        payload[++bufferOffset] = matrix[sourceY][x][2];
                        payload[++bufferOffset] = matrix[sourceY][x][3];
                        payload[++bufferOffset] = matrix[sourceY][x][0];
                    }
                }
            }

            packets[packetIndex] = new DatagramPacket(payload, payload.length, address, DDP_PORT);
        }

        return packets;
    }

    private BufferedImage readImage(String path) throws IOException, BadImageFormatException {
        BufferedImage image = ImageIO.read(new File(path));
        if (image == null) {
            throw new BadImageFormatException("Unsupported image format or file not found.");
        }
        if (image.getHeight() < HEIGHT) {
            throw new BadImageFormatException("Image height must be at least " + HEIGHT + " pixels.");
        }
        return image;
    }

    private void validateGifCanvasDimensions(ImageReader imageReader) throws IOException, BadImageFormatException {
        IIOMetadata streamMetadata = imageReader.getStreamMetadata();
        if (streamMetadata == null) {
            throw new BadImageFormatException("GIF stream metadata is unavailable.");
        }

        IIOMetadataNode root = (IIOMetadataNode) streamMetadata.getAsTree(streamMetadata.getNativeMetadataFormatName());
        IIOMetadataNode logicalScreenDescriptor = getRequiredChild(root, "LogicalScreenDescriptor");

        int logicalScreenWidth = Integer.parseInt(logicalScreenDescriptor.getAttribute("logicalScreenWidth"));
        int logicalScreenHeight = Integer.parseInt(logicalScreenDescriptor.getAttribute("logicalScreenHeight"));
        if (logicalScreenWidth != WIDTH || logicalScreenHeight != HEIGHT) {
            throw new BadImageFormatException(
                    "GIF logical screen must be exactly " + WIDTH + "x" + HEIGHT + " pixels.");
        }
    }

    private FrameData buildCompositedGifFrame(
            ImageReader imageReader,
            int frameIndex,
            GifCompositeState compositeState)
            throws IOException, BadImageFormatException {
        applyGifDisposal(compositeState);

        BufferedImage frame = imageReader.read(frameIndex);
        GifFrameMetadata metadata = readGifFrameMetadata(imageReader.getImageMetadata(frameIndex));
        validateGifFrameBounds(frame, metadata, frameIndex);

        if ("restoreToPrevious".equals(metadata.disposalMethod)) {
            compositeState.restoreToPreviousSnapshot = copyImage(compositeState.canvas);
        } else {
            compositeState.restoreToPreviousSnapshot = null;
        }

        Graphics2D graphics = compositeState.canvas.createGraphics();
        try {
            graphics.setComposite(AlphaComposite.SrcOver);
            graphics.drawImage(frame, metadata.left, metadata.top, null);
        } finally {
            graphics.dispose();
        }

        compositeState.previousFrameMetadata = metadata;
        return new FrameData(getImageData(compositeState.canvas), metadata.displayDurationSeconds);
    }

    private void applyGifDisposal(GifCompositeState compositeState) {
        if (compositeState.previousFrameMetadata == null) {
            return;
        }

        String disposalMethod = compositeState.previousFrameMetadata.disposalMethod;
        if ("restoreToBackgroundColor".equals(disposalMethod)) {
            clearRect(
                    compositeState.canvas,
                    compositeState.previousFrameMetadata.left,
                    compositeState.previousFrameMetadata.top,
                    compositeState.previousFrameMetadata.width,
                    compositeState.previousFrameMetadata.height);
        } else if ("restoreToPrevious".equals(disposalMethod) && compositeState.restoreToPreviousSnapshot != null) {
            Graphics2D graphics = compositeState.canvas.createGraphics();
            try {
                graphics.setComposite(AlphaComposite.Src);
                graphics.drawImage(compositeState.restoreToPreviousSnapshot, 0, 0, null);
            } finally {
                graphics.dispose();
            }
        }
    }

    private GifFrameMetadata readGifFrameMetadata(IIOMetadata metadata) throws BadImageFormatException {
        IIOMetadataNode root = (IIOMetadataNode) metadata.getAsTree(metadata.getNativeMetadataFormatName());
        IIOMetadataNode imageDescriptor = getRequiredChild(root, "ImageDescriptor");
        IIOMetadataNode graphicControlExtension = getRequiredChild(root, "GraphicControlExtension");

        return new GifFrameMetadata(
                Integer.parseInt(imageDescriptor.getAttribute("imageLeftPosition")),
                Integer.parseInt(imageDescriptor.getAttribute("imageTopPosition")),
                Integer.parseInt(imageDescriptor.getAttribute("imageWidth")),
                Integer.parseInt(imageDescriptor.getAttribute("imageHeight")),
                graphicControlExtension.getAttribute("disposalMethod"),
                Integer.parseInt(graphicControlExtension.getAttribute("delayTime")) / 100.0);
    }

    private void validateGifFrameBounds(BufferedImage frame, GifFrameMetadata metadata, int frameIndex)
            throws BadImageFormatException {
        if (frame.getWidth() != metadata.width || frame.getHeight() != metadata.height) {
            throw new BadImageFormatException(
                    "GIF frame " + frameIndex + " data size does not match its metadata descriptor.");
        }
        if (metadata.left < 0
                || metadata.top < 0
                || metadata.left + metadata.width > WIDTH
                || metadata.top + metadata.height > HEIGHT) {
            throw new BadImageFormatException(
                    "GIF frame " + frameIndex + " extends outside the " + WIDTH + "x" + HEIGHT + " canvas.");
        }
    }

    private void clearRect(BufferedImage image, int x, int y, int width, int height) {
        Graphics2D graphics = image.createGraphics();
        try {
            graphics.setComposite(AlphaComposite.Clear);
            graphics.fillRect(x, y, width, height);
        } finally {
            graphics.dispose();
        }
    }

    private BufferedImage copyImage(BufferedImage source) {
        BufferedImage copy = new BufferedImage(source.getWidth(), source.getHeight(), BufferedImage.TYPE_INT_ARGB);
        Graphics2D graphics = copy.createGraphics();
        try {
            graphics.setComposite(AlphaComposite.Src);
            graphics.drawImage(source, 0, 0, null);
        } finally {
            graphics.dispose();
        }
        return copy;
    }

    private IIOMetadataNode getRequiredChild(IIOMetadataNode root, String childName) throws BadImageFormatException {
        for (int i = 0; i < root.getLength(); i++) {
            if (root.item(i) instanceof IIOMetadataNode child && childName.equals(child.getNodeName())) {
                return child;
            }
        }
        throw new BadImageFormatException("Missing GIF metadata node: " + childName);
    }

    private byte[][][] getImageData(BufferedImage image) {
        int realWidth = image.getWidth();
        byte[][][] buffer = new byte[HEIGHT][realWidth][CHANNELS];

        for (int w = 0; w < realWidth; ++w) {
            for (int h = 0; h < HEIGHT; ++h) {
                int hexcode = image.getRGB(w, h);
                int red = (hexcode >> 16) & 0xFF;
                int green = (hexcode >> 8) & 0xFF;
                int blue = hexcode & 0xFF;
                int white = Math.min(Math.min(red, green), blue);

                red -= white;
                green -= white;
                blue -= white;

                buffer[h][w][0] = (byte) (white & 0xFF);
                buffer[h][w][1] = (byte) (red & 0xFF);
                buffer[h][w][2] = (byte) (green & 0xFF);
                buffer[h][w][3] = (byte) (blue & 0xFF);
            }
        }
        return buffer;
    }

    private byte[][][] extractWrappedFrame(byte[][][] imageData, int startX) {
        int realWidth = imageData[0].length;
        byte[][][] frame = new byte[HEIGHT][WIDTH][CHANNELS];

        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                int sourceX = (startX + x) % realWidth;
                copyPixel(imageData, frame, y, sourceX, x);
            }
        }

        return frame;
    }

    private byte[][][] extractClippedFrame(byte[][][] imageData, int startX) {
        int realWidth = imageData[0].length;
        byte[][][] frame = new byte[HEIGHT][WIDTH][CHANNELS];

        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                int sourceX = startX + x;
                if (sourceX < realWidth) {
                    copyPixel(imageData, frame, y, sourceX, x);
                }
            }
        }

        return frame;
    }

    private void copyPixel(byte[][][] source, byte[][][] destination, int y, int sourceX, int destinationX) {
        destination[y][destinationX][0] = source[y][sourceX][0];
        destination[y][destinationX][1] = source[y][sourceX][1];
        destination[y][destinationX][2] = source[y][sourceX][2];
        destination[y][destinationX][3] = source[y][sourceX][3];
    }

    private PreparedPlaybackPackets reportPreparationFailure(String mode, String path, Exception exception) {
        DriverStation.reportError(
                "WLED " + mode + " preparation failed for " + path + ": " + exception.getMessage(),
                false);
        return new PreparedPlaybackPackets(new DatagramPacket[0], 0, new double[0]);
    }

    @FunctionalInterface
    private interface FrameBuilder {
        FrameData buildFrame(int frameIndex) throws IOException, BadImageFormatException;
    }

    private static class BadImageFormatException extends Exception {
        private static final long serialVersionUID = 1L;

        private BadImageFormatException(String message) {
            super(message);
        }
    }
}
