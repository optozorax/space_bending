uniform int sceneType;
uniform float aspectRatio;
uniform float size;
uniform float darknessFactor;
uniform float anim;
uniform int drawPortalSurface;

varying vec3 vNormal;
varying vec3 vViewPosition;
varying vec2 vUv;

// Colors
const vec4 blueColor = vec4(9.0/255.0, 164.0/255.0, 201.0/255.0, 1.0);
const vec4 orangeColor = vec4(237.0/255.0, 108.0/255.0, 13.0/255.0, 1.0);
const vec4 redColor = vec4(136.0/255.0, 0.0/255.0, 21.0/255.0, 1.0);
const vec4 greenColor = vec4(181.0/255.0, 230.0/255.0, 29.0/255.0, 1.0);
const vec4 whiteColor = vec4(1.0, 1.0, 1.0, 1.0);
const vec4 checkerWhite = vec4(1.0, 1.0, 1.0, 40.0/255.0);
const vec4 checkerBlack = vec4(0.0, 0.0, 0.0, 40.0/255.0);

// Convert UV coordinates based on aspect ratio
vec2 adjustUV(vec2 uv) {
    vec2 adjusted = uv;
    if (aspectRatio > 1.0) {
        adjusted.x *= aspectRatio;
    } else if (aspectRatio < 1.0) {
        adjusted.y /= aspectRatio;
    }
    return adjusted;
}

// SDF (Signed Distance Field) functions
float circleSDF(vec2 p, vec2 center, float radius) {
    return length(p - center) - radius;
}

float lineSDF(vec2 p, vec2 a, vec2 b, float thickness) {
    vec2 pa = p - a;
    vec2 ba = b - a;
    float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    return length(pa - ba * h) - thickness * 0.5;
}

float rotatedSquareSDF(vec2 p, vec2 center, float size, float angle) {
    // Translate
    vec2 p2 = p - center;
    
    // Rotate
    float c = cos(-angle);
    float s = sin(-angle);
    p2 = vec2(c * p2.x - s * p2.y, s * p2.x + c * p2.y);
    
    // Square SDF
    vec2 d = abs(p2) - vec2(size * 0.5);
    return min(max(d.x, d.y), 0.0) + length(max(d, 0.0));
}

// Drawing functions
vec4 drawCircle(vec2 uv, vec2 center, float radius, vec4 color) {
    float dist = circleSDF(uv, center, radius);
    return dist <= 0.0 ? color : vec4(0.0);
}

vec4 drawLine(vec2 uv, vec2 start, vec2 end, float thickness, vec4 color) {
    float dist = lineSDF(uv, start, end, thickness);
    return dist <= 0.0 ? color : vec4(0.0);
}

vec4 drawRotatedSquare(vec2 uv, vec2 center, float size, float angle, vec4 color) {
    float dist = rotatedSquareSDF(uv, center, size, angle);
    return dist <= 0.0 ? color : vec4(0.0);
}

vec4 drawCheckerboard(vec2 uv, float tileSize, vec4 color1, vec4 color2) {
    // Scale coordinates to create tiles of the specified size
    vec2 tiledUV = floor(uv / tileSize);
    
    // Determine tile color
    bool isEven = mod(tiledUV.x + tiledUV.y, 2.0) == 0.0;
    
    return isEven ? color1 : color2;
}

// Alpha blending function
vec4 alphaBlend(vec4 src, vec4 dst) {
    float outAlpha = src.a + dst.a * (1.0 - src.a);
    if (outAlpha <= 0.0) return vec4(0.0);
    
    vec3 outColor = (src.rgb * src.a + dst.rgb * dst.a * (1.0 - src.a)) / outAlpha;
    return vec4(outColor, outAlpha);
}

// Portal1 scene
vec4 drawPortal1Scene(vec2 uv) {
    float blueX = 0.2;
    float orangeX = 0.8;
    float startY = 0.4;
    float endY = 0.6;
    float thickness = 0.005;
    
    // Start with white background
    vec4 result = whiteColor;
    
    // Draw checkerboard patterns
    vec4 checker2 = drawCheckerboard(uv, 1.0 / size, checkerWhite, checkerBlack);
    result = alphaBlend(checker2, result);
    
    // Draw green squares
    float sq1Size = 0.1;
    float sq1Angle = 3.14159 / 4.0 + anim;
    vec4 sq1Color = vec4(0.0, 1.0, 0.0, 192.0/255.0);
    
    // Only draw if uv.x is less than center.x (discardRight = true)
    if (uv.x <= blueX) {
        vec4 square1 = drawRotatedSquare(uv, vec2(blueX + anim*0.1, (startY + endY) / 2.0), 
                                        sq1Size, sq1Angle, sq1Color);
        result = alphaBlend(square1, result);
    }
    
    // Only draw if uv.x is greater than center.x (discardLeft = true)
    if (uv.x >= orangeX) {
        vec4 square2 = drawRotatedSquare(uv, vec2(orangeX + anim*0.1, (startY + endY) / 2.0), 
                                        sq1Size, sq1Angle, sq1Color);
        result = alphaBlend(square2, result);
    }
    
    // Draw blue squares
    float sq2Size = 0.09;
    float sq2Angle = 3.14159 / 3.0;
    vec4 sq2Color = vec4(128.0/255.0, 128.0/255.0, 1.0, 192.0/255.0);
    
    // Only draw if uv.x is greater than center.x (discardLeft = true)
    if (uv.x >= blueX) {
        vec4 square3 = drawRotatedSquare(uv, vec2(blueX - 0.02 - anim*0.3, (startY + endY) / 2.0 + 0.01), 
                                        sq2Size, sq2Angle, sq2Color);
        result = alphaBlend(square3, result);
    }
    
    // Only draw if uv.x is less than center.x (discardRight = true)
    if (uv.x <= orangeX) {
        vec4 square4 = drawRotatedSquare(uv, vec2(orangeX - 0.02 - anim*0.3, (startY + endY) / 2.0 + 0.01), 
                                        sq2Size, sq2Angle, sq2Color);
        result = alphaBlend(square4, result);
    }
    
    // Draw blue portal
    vec4 blueCircle1 = drawCircle(uv, vec2(blueX, startY), thickness * 1.5, blueColor);
    result = alphaBlend(blueCircle1, result);
    
    vec4 blueCircle2 = drawCircle(uv, vec2(blueX, endY), thickness * 1.5, blueColor);
    result = alphaBlend(blueCircle2, result);
    
    if (drawPortalSurface == 1) {
        vec4 blueLine = drawLine(uv, vec2(blueX, startY), vec2(blueX, endY), thickness, blueColor);
        result = alphaBlend(blueLine, result);
    }
    
    // Draw orange portal
    vec4 orangeCircle1 = drawCircle(uv, vec2(orangeX, startY), thickness * 1.5, orangeColor);
    result = alphaBlend(orangeCircle1, result);
    
    vec4 orangeCircle2 = drawCircle(uv, vec2(orangeX, endY), thickness * 1.5, orangeColor);
    result = alphaBlend(orangeCircle2, result);
    
    if (drawPortalSurface == 1) {
        vec4 orangeLine = drawLine(uv, vec2(orangeX, startY), vec2(orangeX, endY), thickness, orangeColor);
        result = alphaBlend(orangeLine, result);
    }
    
    return result;
}

// Cylinder scene
vec4 drawCylinderScene(vec2 uv) {
    float thickness = 0.02;
    
    // Start with white background
    vec4 result = whiteColor;
    
    // Draw checkerboard patterns
    vec4 checker2 = drawCheckerboard(uv, 1.0 / size, checkerWhite, checkerBlack);
    result = alphaBlend(checker2, result);
    
    // Draw green squares
    float sq1Size = 0.3;
    float sq1Angle = 3.14159 / 3.0;
    vec4 sq1Color = vec4(0.0, 1.0, 0.0, 192.0/255.0);
    
    vec4 square1 = drawRotatedSquare(uv, vec2(2.0, 0.4), sq1Size, sq1Angle, sq1Color);
    result = alphaBlend(square1, result);
    
    vec4 square2 = drawRotatedSquare(uv, vec2(0.0, 0.4), sq1Size, sq1Angle, sq1Color);
    result = alphaBlend(square2, result);
    
    // Draw blue square
    float sq2Size = 0.4;
    float sq2Angle = 3.14159 / 1.5;
    vec4 sq2Color = vec4(128.0/255.0, 128.0/255.0, 1.0, 192.0/255.0);
    
    vec4 square3 = drawRotatedSquare(uv, vec2(0.7, 0.9), sq2Size, sq2Angle, sq2Color);
    result = alphaBlend(square3, result);
    
    // Draw lines
    if (drawPortalSurface == 1) {
        vec4 blueLine = drawLine(uv, vec2(0.0, 0.0), vec2(0.0, 1.0), thickness, blueColor);
        result = alphaBlend(blueLine, result);
        
        vec4 orangeLine = drawLine(uv, vec2(2.0, 0.0), vec2(2.0, 1.0), thickness, orangeColor);
        result = alphaBlend(orangeLine, result);
    }
    
    return result;
}

// Torus scene
vec4 drawTorusScene(vec2 uv) {
    float thickness = 0.02;
    
    // Start with white background
    vec4 result = whiteColor;
    
    // Draw checkerboard patterns
    vec4 checker2 = drawCheckerboard(uv, 1.0 / size, checkerWhite, checkerBlack);
    result = alphaBlend(checker2, result);
    
    // Draw green squares
    float sq1Size = 0.3;
    float sq1Angle = 3.14159 / 3.0;
    vec4 sq1Color = vec4(0.0, 1.0, 0.0, 192.0/255.0);
    
    vec4 square1 = drawRotatedSquare(uv, vec2(3.0, 0.4), sq1Size, sq1Angle, sq1Color);
    result = alphaBlend(square1, result);
    
    vec4 square2 = drawRotatedSquare(uv, vec2(0.0, 0.4), sq1Size, sq1Angle, sq1Color);
    result = alphaBlend(square2, result);
    
    // Draw blue squares
    float sq2Size = 0.4;
    float sq2Angle = 3.14159 / 1.5;
    vec4 sq2Color = vec4(128.0/255.0, 128.0/255.0, 1.0, 192.0/255.0);
    
    vec4 square3 = drawRotatedSquare(uv, vec2(0.7, 0.9), sq2Size, sq2Angle, sq2Color);
    result = alphaBlend(square3, result);
    
    vec4 square4 = drawRotatedSquare(uv, vec2(0.7, -0.1), sq2Size, sq2Angle, sq2Color);
    result = alphaBlend(square4, result);
    
    if (drawPortalSurface == 1) {
        // Draw vertical lines
        vec4 blueLine = drawLine(uv, vec2(0.0, 0.0), vec2(0.0, 1.0), thickness, blueColor);
        result = alphaBlend(blueLine, result);
        
        vec4 orangeLine = drawLine(uv, vec2(3.0, 0.0), vec2(3.0, 1.0), thickness, orangeColor);
        result = alphaBlend(orangeLine, result);
        
        // Draw horizontal lines
        vec4 redLine = drawLine(uv, vec2(0.0, 0.0), vec2(3.0, 0.0), thickness, redColor);
        result = alphaBlend(redLine, result);
        
        vec4 greenLine = drawLine(uv, vec2(0.0, 1.0), vec2(3.0, 1.0), thickness, greenColor);
        result = alphaBlend(greenLine, result);
    }
    
    return result;
}

// Flat scene
vec4 drawFlatScene(vec2 uv) {
    // Start with white background
    vec4 result = whiteColor;
    
    // Draw checkerboard patterns
    vec4 checker2 = drawCheckerboard(uv, 1.0 / size, checkerWhite, checkerBlack);
    result = alphaBlend(checker2, result);
    
    return result;
}

void main() {
    // Normalize view direction
    vec3 viewDir = normalize(vViewPosition);
    
    // Calculate dot product between normal and view direction
    float dotNV = max(0.0, abs(dot(vNormal, viewDir)));
    
    // Calculate darkness based on angle
    float darkness = (1.0 - dotNV) * darknessFactor;
    
    // Convert UV coordinates to texture space based on aspect ratio
    vec2 texUV = adjustUV(vUv);
    
    // Draw appropriate scene based on scene type
    vec4 texColor;
    if (sceneType == 0) {
        texColor = drawPortal1Scene(texUV);
    } else if (sceneType == 1) {
        texColor = drawCylinderScene(texUV);
    } else if (sceneType == 2) {
        texColor = drawTorusScene(texUV);
    } else {
        texColor = drawFlatScene(texUV);
    }
    
    // Apply darkness to the texture color
    vec3 finalColor = texColor.rgb * (1.0 - darkness);
    
    gl_FragColor = vec4(finalColor, texColor.a);
}
