package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.DbgLog;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;

public class VuforiaStuff {

    VuforiaLocalizer vuforia;

    public VuforiaStuff(VuforiaLocalizer vuforia) {
        this.vuforia = vuforia;
    }

    public enum skystonePos {
        LEFT, CENTER, RIGHT;
    }

    public skystonePos vuforiascan(boolean saveBitmaps, boolean red) {
        Image rgbImage = null;
        int rgbTries = 0;
        /*
        double colorcountL = 0;
        double colorcountC = 0;
        double colorcountR = 0;
        */
        double yellowCountL = 1;
        double yellowCountC = 1;
        double yellowCountR = 1;

        double blackCountL = 1;
        double blackCountC = 1;
        double blackCountR = 1;
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        VuforiaLocalizer.CloseableFrame closeableFrame = null;
        this.vuforia.setFrameQueueCapacity(1);
        while (rgbImage == null) {
            try {
                closeableFrame = this.vuforia.getFrameQueue().take();
                long numImages = closeableFrame.getNumImages();

                for (int i = 0; i < numImages; i++) {
                    if (closeableFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                        rgbImage = closeableFrame.getImage(i);
                        if (rgbImage != null) {
                            break;
                        }
                    }
                }
            } catch (InterruptedException exc) {

            } finally {
                if (closeableFrame != null) closeableFrame.close();
            }
        }

        if (rgbImage != null) {

            // copy the bitmap from the Vuforia frame
            Bitmap bitmap = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(rgbImage.getPixels());

            String path = Environment.getExternalStorageDirectory().toString();
            FileOutputStream out = null;

            String bitmapName;
            String croppedBitmapName;

            if (red) {
                bitmapName = "BitmapRED.png";
                croppedBitmapName = "BitmapCroppedRED.png";
            } else {
                bitmapName = "BitmapBLUE.png";
                croppedBitmapName = "BitmapCroppedBLUE.png";
            }

            //Save bitmap to file
            if (saveBitmaps) {
                try {
                    File file = new File(path, bitmapName);
                    out = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                } catch (Exception e) {
                    e.printStackTrace();
                } finally {
                    try {
                        if (out != null) {
                            out.flush();
                            out.close();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }

            int cropStartX;
            int cropStartY;
            int cropWidth;
            int cropHeight;

            /*
                Possible Changes Needed : Source changed their code 8 days agao (1/18/20)
               GitHub Source: https://github.com/Dkjam5511/Skystone

            if (red) {
                cropStartX = (int) ((140.0 / 720.0) * bitmap.getWidth());
                cropStartX = (int) ((150.0 / 720.0) * bitmap.getWidth());
                cropStartY = (int) ((100.0 / 480.0) * bitmap.getHeight());
                cropWidth = (int) ((550.0 / 720.0) * bitmap.getWidth());
                cropHeight = (int) ((130.0 / 480.0) * bitmap.getHeight());
                cropWidth = (int) ((530.0 / 720.0) * bitmap.getWidth());
                cropHeight = (int) ((150.0 / 480.0) * bitmap.getHeight());
            } else {
                cropStartX = (int) ((370.0 / 1280.0) * bitmap.getWidth());
                cropStartY = (int) ((170.0 / 720.0) * bitmap.getHeight());
                cropWidth = (int) ((890.0 / 1280.0) * bitmap.getWidth());
                cropHeight = (int) ((125.0 / 720.0) * bitmap.getHeight());


                Take Note of the cropStartX numbers... Should test to see if that makes it better/worse in detection
             */


            if (red) {      // Assume your starting on the RED Alliance
                cropStartX = (int) ((120.0 / 720.0) * bitmap.getWidth());           // 1/29/20    Was ((120.0 / 720)    changing to ((150.0 / 720)
                cropStartY = (int) ((100.0 / 480.0) * bitmap.getHeight());
                cropWidth = (int) ((590.0 / 720.0) * bitmap.getWidth());            // 1/29/20    Was ((590.0 / 720)    changing to ((530.0 / 720)
                cropHeight = (int) ((170.0 / 480.0) * bitmap.getHeight());          // 1/29/20    Was ((170.0 / 720)    changing to ((150.0 / 720)
            } else {      // Assume your starting the the Blue Alliance
                cropStartX = (int) ((370.0 / 1280.0) * bitmap.getWidth());
                cropStartY = (int) ((130.0 / 720.0) * bitmap.getHeight());
                cropWidth = (int) ((890.0 / 1280.0) * bitmap.getWidth());
                cropHeight = (int) ((165.0 / 720.0) * bitmap.getHeight());          // 1/29/20    Was ((165.0 / 720)    changing to ((125.0 / 720)
            }

            DbgLog.msg("9977 vuforiascan"
                    + " cropStartX: " + cropStartX
                    + " cropStartY: " + cropStartY
                    + " cropWidth: " + cropWidth
                    + " cropHeight: " + cropHeight
                    + " Width: " + bitmap.getWidth()
                    + " Height: " + bitmap.getHeight()
            );


            bitmap = createBitmap(bitmap, cropStartX, cropStartY, cropWidth, cropHeight); //Cropped Bitmap to show only stones

            // Save cropped bitmap to file
            if (saveBitmaps) {
                try {
                    File file = new File(path, croppedBitmapName);
                    out = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                } catch (Exception e) {
                    e.printStackTrace();
                } finally {
                    try {
                        if (out != null) {
                            out.flush();
                            out.close();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }
            bitmap = createScaledBitmap(bitmap, 110, 20, true); //Compress bitmap to reduce scan time


            int height;
            int width;
            int pixel;
            int bitmapWidth = bitmap.getWidth();
            int bitmapHeight = bitmap.getHeight();
            int colWidth = (int) ((double) bitmapWidth / 6.0);
            int colorLStartCol = (int) ((double) bitmapWidth * (1.0 / 6.0) - ((double) colWidth / 2.0));
            int colorCStartCol = (int) ((double) bitmapWidth * (3.0 / 6.0) - ((double) colWidth / 2.0));
            int colorRStartCol = (int) ((double) bitmapWidth * (5.0 / 6.0) - ((double) colWidth / 2.0));

            for (height = 0; height < bitmapHeight; ++height) {
                for (width = colorLStartCol; width < colorLStartCol + colWidth; ++width) {
                    pixel = bitmap.getPixel(width, height);
                    if (Color.red(pixel) < 200 || Color.green(pixel) < 200 || Color.blue(pixel) < 200) {
                        yellowCountL += Color.red(pixel);
                        blackCountL += Color.blue(pixel);
                    }

                }
                for (width = colorCStartCol; width < colorCStartCol + colWidth; ++width) {
                    pixel = bitmap.getPixel(width, height);

                    if (Color.red(pixel) < 200 || Color.green(pixel) < 200 || Color.blue(pixel) < 200) {
                        yellowCountC += Color.red(pixel);
                        blackCountC += Color.blue(pixel);
                    }

                }

                for (width = colorRStartCol; width < colorRStartCol + colWidth; ++width) {
                    pixel = bitmap.getPixel(width, height);

                    if (Color.red(pixel) < 200 || Color.green(pixel) < 200 || Color.blue(pixel) < 200) {
                        yellowCountR += Color.red(pixel);
                        blackCountR += Color.blue(pixel);
                    }

                }
            }
        }

        double blackYellowRatioL = blackCountL / yellowCountL;
        double blackYellowRatioC = blackCountC / yellowCountC;
        double blackYellowRatioR = blackCountR / yellowCountR;


        skystonePos pos;

        // This works, however, we do not have a graceful way to exit if it cant detect the
        // right stone.  For example, if it only detects 3 gold blocks (usually will be impossible and invalid field setup)
        // but I think we should try to account for it

        if (blackYellowRatioL > blackYellowRatioC && blackYellowRatioL > blackYellowRatioR) {
            pos = skystonePos.LEFT;

        } else if (blackYellowRatioC > blackYellowRatioL && blackYellowRatioC > blackYellowRatioR) {
            pos = skystonePos.CENTER;
        } else {
         pos = skystonePos.RIGHT;

        }


        DbgLog.msg("black/yellow L: " + blackCountL + "/" + yellowCountL);
        DbgLog.msg("black/yellow C: " + blackCountC + "/" + yellowCountC);
        DbgLog.msg("black/yellow R: " + blackCountR + "/" + yellowCountR);
        DbgLog.msg("-------------------");
        DbgLog.msg("blackYellowRatioL is: " + blackYellowRatioL);
        DbgLog.msg("blackYellowRatioC is: " + blackYellowRatioC);
        DbgLog.msg("blackYellowRatioR is: " + blackYellowRatioR);
        DbgLog.msg("skystone Postistion is: " + pos);
        DbgLog.msg("-------------------");

        return pos;
    }

}