"""
Ceci est un exercice pour presenter l'utilisation de \
    l'alebre lineaire dans un plug-ins python \
        vous pouvez a partir de ce plug-in creer une chaine de 3 joints \
            ces joints seront positionner en fonction de la position de \
                3 locators en input du node
"""

import maya.OpenMayaMPx as OpenMayaMPx
import maya.OpenMaya as OpenMaya
import maya.api.OpenMaya as OpenMaya2

import math


class DvSlerpMatPlug(OpenMayaMPx.MPxNode):

    # Define node properties.
    kname = "dvSlerpMatPlug"
    kplugin_id = OpenMaya.MTypeId(0x90000005)

    # Define node attributes.
    rigMode = OpenMaya.MObject()
    matrixA = OpenMaya.MObject()
    matrixB = OpenMaya.MObject()
    weights = OpenMaya.MObject()
    parentInverseMatrix = OpenMaya.MObject()

    offsetMatrix = OpenMaya.MObject()

    # OUTPUTS
    xform = OpenMaya.MObject()

    translate = OpenMaya.MObject()
    translateX = OpenMaya.MObject()
    translateY = OpenMaya.MObject()
    translateZ = OpenMaya.MObject()

    rotate = OpenMaya.MObject()
    rotateX = OpenMaya.MObject()
    rotateY = OpenMaya.MObject()
    rotateZ = OpenMaya.MObject()

    scale = OpenMaya.MObject()
    scaleX = OpenMaya.MObject()
    scaleY = OpenMaya.MObject()
    scaleZ = OpenMaya.MObject()


    def __init__(self):
        OpenMayaMPx.MPxNode.__init__(self)


    def slerp(self, qa, qb, t):
        """Calculates the quaternion slerp between two quaternions.
    
        From: http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
    
        :param qa: Start MQuaternion.
        :param qb: End MQuaternion.
        :param t: Parameter between 0.0 and 1.0
        :return: An MQuaternion interpolated between qa and qb.
        """
        qm = OpenMaya.MQuaternion()
    
        # Calculate angle between them.
        cos_half_theta = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z
        # if qa == qb or qa == -qb then theta = 0 and we can return qa
        if abs(cos_half_theta) >= 1.0:
            qm.w = qa.w
            qm.x = qa.x
            qm.y = qa.y
            qm.z = qa.z
            return qa
    
        # Calculate temporary values
        half_theta = math.acos(cos_half_theta)
        sin_half_theta = math.sqrt(1.0 - cos_half_theta * cos_half_theta)
        # if theta = 180 degrees then result is not fully defined
        # we could rotate around any axis normal to qa or qb
        if math.fabs(sin_half_theta) < 0.001:
            qm.w = (qa.w * 0.5 + qb.w * 0.5)
            qm.x = (qa.x * 0.5 + qb.x * 0.5)
            qm.y = (qa.y * 0.5 + qb.y * 0.5)
            qm.z = (qa.z * 0.5 + qb.z * 0.5)
            return qm
    
        ratio_a = math.sin((1 - t) * half_theta) / sin_half_theta
        ratio_b = math.sin(t * half_theta) / sin_half_theta
        # Calculate quaternion
        qm.w = (qa.w * ratio_a + qb.w * ratio_b)
        qm.x = (qa.x * ratio_a + qb.x * ratio_b)
        qm.y = (qa.y * ratio_a + qb.y * ratio_b)
        qm.z = (qa.z * ratio_a + qb.z * ratio_b)
        return qm

    def decompose_matrix(self, mMatrix):
        # Convertir MMatrix en MTransformMatrix
        mTrsfmMtx = OpenMaya.MTransformationMatrix(mMatrix)

        # Valeurs translation
        trans = mTrsfmMtx.translation(OpenMaya.MSpace.kWorld)

        # Valeur rotation Euler en radian
        quat = mTrsfmMtx.rotation()
        angles = quat.asEulerRotation()

        # Extraire scale.
        scale = [1.0, 1.0, 1.0]
        
        scaleDoubleArray = OpenMaya.MScriptUtil()
        scaleDoubleArray.createFromList( [0.0, 0.0, 0.0], 3 )
        scaleDoubleArrayPtr = scaleDoubleArray.asDoublePtr()
        
        mTrsfmMtx.getScale(scaleDoubleArrayPtr,OpenMaya.MSpace.kObject)
        scale[0] = OpenMaya.MScriptUtil().getDoubleArrayItem(scaleDoubleArrayPtr,0)
        scale[1] = OpenMaya.MScriptUtil().getDoubleArrayItem(scaleDoubleArrayPtr,1)
        scale[2] = OpenMaya.MScriptUtil().getDoubleArrayItem(scaleDoubleArrayPtr,2)
        
        return [trans.x, trans.y, trans.z], [angles.x, angles.y, angles.z], scale


    def compute(self, plug, data):

        # Read plugs.
        rigMode_state = data.inputValue(
                DvSlerpMatPlug.rigMode
            ).asBool()

        blend_value = data.inputValue(
                DvSlerpMatPlug.blend
            ).asFloat()

        matrixA_mMatrix = data.inputValue(
                DvSlerpMatPlug.matrixA
            ).asMatrix()

        matrixB_mMatrix = data.inputValue(
                DvSlerpMatPlug.matrixB
            ).asMatrix()

        mTrsfmMtxB = OpenMaya.MTransformationMatrix(matrixB_mMatrix)
        posB = mTrsfmMtxB.getTranslation(OpenMaya.MSpace.kWorld);

        weights = data.inputValue(
                DvSlerpMatPlug.weights
            ).asDouble2()
        print "weights :: ", weights

        parentInverseMatrix_mMatrix = data.inputValue(
                DvSlerpMatPlug.parentInverseMatrix
            ).asMatrix()

        if rigMode_state:
            offset_handle = data.outputValue(
                    DvSlerpMatPlug.offsetMatrix
                )
            offsetMatrix_mMatrix = matrixA_mMatrix * matrixB_mMatrix.inverse()
            offset_handle.setMMatrix(offsetMatrix_mMatrix)
        
        offsetMatrix_mMatrix = data.inputValue(
                DvSlerpMatPlug.offsetMatrix
            ).asMatrix()
        
        offsetedMatrix = offsetMatrix_mMatrix * matrixB_mMatrix;

        mTrsfmMtxA = OpenMaya.MTransformationMatrix(matrixA_mMatrix)
        mTrsfmMtxA.setTranslation(
                OpenMaya.MVector(0.0, 0.0, 0.0),
                OpenMaya.MSpace.kWorld
            )
        quatA = mTrsfmMtxA.rotation()

        mTrsfmMtxB = OpenMaya.MTransformationMatrix(offsetedMatrix)
        mTrsfmMtxB.setTranslation(
                OpenMaya.MVector(0.0, 0.0, 0.0),
                OpenMaya.MSpace.kWorld
            )
        quatB = mTrsfmMtxB.rotation()

        quatC = self.slerp(quatA, quatB, blend_value)

        matC = quatC.asMatrix()
        mTrsfmMtxC = OpenMaya.MTransformationMatrix(matC)
        mTrsfmMtxC.setTranslation(posB, OpenMaya.MSpace.kWorld)
        matC = mTrsfmMtxC.asMatrix()

        final_mMatrix =  matC * parentInverseMatrix_mMatrix

        transforms = self.decompose_matrix(final_mMatrix)
        
        # OUTPUTS
        xform_handle = data.outputValue(self.xform)
        
        # Set output shoulder
        out_tr = xform_handle.child(
                DvSlerpMatPlug.translate
            )
        out_tr.set3Double(
                transforms[0][0],
                transforms[0][1],
                transforms[0][2]
            )

        out_rot = xform_handle.child(
                DvSlerpMatPlug.rotate
            )
        out_rot.set3Double(
                transforms[1][0],
                transforms[1][1],
                transforms[1][2]
            )

        out_scl = xform_handle.child(
                DvSlerpMatPlug.scale
            )
        out_scl.set3Double(
                transforms[2][0],
                transforms[2][1] + ((1-weights[0]) * transforms[1][2]),
                transforms[2][2] + ((1-weights[1]) * transforms[1][1])
            )

        xform_handle.setClean()
        
        data.setClean(plug)

        return True


def creator():
    return OpenMayaMPx.asMPxPtr(DvSlerpMatPlug())


def initialize():
    nAttr = OpenMaya.MFnNumericAttribute()
    mAttr = OpenMaya.MFnMatrixAttribute()
    cAttr = OpenMaya.MFnCompoundAttribute()
    uAttr = OpenMaya.MFnUnitAttribute()

    # INPUTS
    DvSlerpMatPlug.rigMode = nAttr.create(
            "rigMode",
            "rigmode",
            OpenMaya.MFnNumericData.kBoolean,
            True
        )
    nAttr.setWritable(True)
    nAttr.setStorable(True)
    DvSlerpMatPlug.addAttribute(DvSlerpMatPlug.rigMode)

    DvSlerpMatPlug.blend = nAttr.create(
            "blend",
            "blnd",
            OpenMaya.MFnNumericData.kFloat,
            .5
        )
    nAttr.setWritable(True)
    nAttr.setStorable(True)
    nAttr.setMin(0.0)
    nAttr.setMax(1.0)
    DvSlerpMatPlug.addAttribute(DvSlerpMatPlug.blend)

    DvSlerpMatPlug.weights = nAttr.create(
            "weights",
            "weights",
            OpenMaya.MFnNumericData.k2Double,
            1.0
        )
    nAttr.setWritable(True)
    nAttr.setStorable(True)
    DvSlerpMatPlug.addAttribute(DvSlerpMatPlug.weights)

    DvSlerpMatPlug.matrixA = mAttr.create(
            "matrixA",
            "matA"
        )
    mAttr.setStorable(True)
    DvSlerpMatPlug.addAttribute(DvSlerpMatPlug.matrixA)

    DvSlerpMatPlug.matrixB = mAttr.create(
            "matrixB",
            "matB"
        )
    mAttr.setStorable(False)
    DvSlerpMatPlug.addAttribute(DvSlerpMatPlug.matrixB)

    DvSlerpMatPlug.parentInverseMatrix = mAttr.create(
            "parentInverseMatrix",
            "pim"
        )
    mAttr.setStorable(False)
    DvSlerpMatPlug.addAttribute(DvSlerpMatPlug.parentInverseMatrix)

    DvSlerpMatPlug.offsetMatrix = mAttr.create(
            "offsetMatrix",
            "offm"
        )
    mAttr.setStorable(False)
    DvSlerpMatPlug.addAttribute(DvSlerpMatPlug.offsetMatrix)

    # OUTPUTS
    # Translate
    DvSlerpMatPlug.translateX = nAttr.create(
        "translateX",
        "tx",
        OpenMaya.MFnNumericData.kDouble,
        0.0
    )
    DvSlerpMatPlug.translateY = nAttr.create(
        "translateY",
        "ty",
        OpenMaya.MFnNumericData.kDouble,
        0.0
    )
    DvSlerpMatPlug.translateZ = nAttr.create(
        "translateZ",
        "tz",
        OpenMaya.MFnNumericData.kDouble,
        0.0
    )
    DvSlerpMatPlug.translate = nAttr.create(
        "translate",   "t",
        DvSlerpMatPlug.translateX,
        DvSlerpMatPlug.translateY,
        DvSlerpMatPlug.translateZ
    )
    nAttr.setStorable(False)

    # Rotate
    DvSlerpMatPlug.rotateX = uAttr.create(
        "rotateX",
        "rx",
        OpenMaya.MFnUnitAttribute.kAngle,
        0.0
    )
    DvSlerpMatPlug.rotateY = uAttr.create(
        "rotateY",
        "ry",
        OpenMaya.MFnUnitAttribute.kAngle,
        0.0
    )
    DvSlerpMatPlug.rotateZ = uAttr.create(
        "rotateZ",
        "rz",
        OpenMaya.MFnUnitAttribute.kAngle,
        0.0
    )
    DvSlerpMatPlug.rotate = nAttr.create(
        "rotate",   "r",
        DvSlerpMatPlug.rotateX,
        DvSlerpMatPlug.rotateY,
        DvSlerpMatPlug.rotateZ
    )
    nAttr.setStorable(False)

    # Scale
    DvSlerpMatPlug.scaleX = nAttr.create(
        "scaleX",
        "sx",
        OpenMaya.MFnNumericData.kDouble,
        0.0
    )
    DvSlerpMatPlug.scaleY = nAttr.create(
        "scaleY",
        "sy",
        OpenMaya.MFnNumericData.kDouble,
        0.0
    )
    DvSlerpMatPlug.scaleZ = nAttr.create(
        "scaleZ",
        "sz",
        OpenMaya.MFnNumericData.kDouble,
        0.0
    )
    DvSlerpMatPlug.scale = nAttr.create(
        "scale",   "s",
        DvSlerpMatPlug.scaleX,
        DvSlerpMatPlug.scaleY,
        DvSlerpMatPlug.scaleZ
    )
    nAttr.setStorable(False)

    DvSlerpMatPlug.xform = cAttr.create("xform", "xf")
    cAttr.addChild(DvSlerpMatPlug.translate)
    cAttr.addChild(DvSlerpMatPlug.rotate)
    cAttr.addChild(DvSlerpMatPlug.scale)
    DvSlerpMatPlug.addAttribute(DvSlerpMatPlug.xform)

    # Attribut affect
    DvSlerpMatPlug.attributeAffects(
            DvSlerpMatPlug.rigMode,
            DvSlerpMatPlug.xform
        )

    DvSlerpMatPlug.attributeAffects(
            DvSlerpMatPlug.weights,
            DvSlerpMatPlug.xform
        )

    DvSlerpMatPlug.attributeAffects(
            DvSlerpMatPlug.blend,
            DvSlerpMatPlug.xform
        )

    DvSlerpMatPlug.attributeAffects(
            DvSlerpMatPlug.matrixA,
            DvSlerpMatPlug.xform
        )

    DvSlerpMatPlug.attributeAffects(
            DvSlerpMatPlug.matrixB,
            DvSlerpMatPlug.xform
        )

    DvSlerpMatPlug.attributeAffects(
            DvSlerpMatPlug.parentInverseMatrix,
            DvSlerpMatPlug.xform
        )


def initializePlugin(obj):
    plugin = OpenMayaMPx.MFnPlugin(obj, "damsOLabo", "1.0", "Any")
    try:
        plugin.registerNode(
                DvSlerpMatPlug.kname,
                DvSlerpMatPlug.kplugin_id,
                creator,
                initialize
            )
    except:
        raise RuntimeError, "Failed to register node: '{}'".format(DvSlerpMatPlug.kname)


def uninitializePlugin(obj):
    plugin = OpenMayaMPx.MFnPlugin(obj)
    try:
        plugin.deregisterNode(DvSlerpMatPlug.kplugin_id)
    except:
        raise RuntimeError, "Failed to register node: '{}'".format(DvSlerpMatPlug.kname)
