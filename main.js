import * as THREE from 'https://cdn.skypack.dev/three@0.145.0';
import { EffectComposer } from 'https://unpkg.com/three@0.145.0/examples/jsm/postprocessing/EffectComposer.js';
import { RenderPass } from 'https://unpkg.com/three@0.145.0/examples/jsm/postprocessing/RenderPass.js';
import { ShaderPass } from 'https://unpkg.com/three@0.145.0/examples/jsm/postprocessing/ShaderPass.js';
import { SMAAPass } from 'https://unpkg.com/three@0.145.0/examples/jsm/postprocessing/SMAAPass.js';
import { GammaCorrectionShader } from 'https://unpkg.com/three@0.145.0/examples/jsm/shaders/GammaCorrectionShader.js';
import { EffectShader } from "./EffectShader.js";
import { OrbitControls } from 'https://unpkg.com/three@0.145.0/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from "https://unpkg.com/three@0.145.0/examples/jsm/loaders/GLTFLoader.js";
import { SimplifyModifier } from 'https://unpkg.com/three@0.145.0/examples/jsm/modifiers/SimplifyModifier.js';
import { ConvexGeometry } from 'https://unpkg.com/three@0.145.0/examples/jsm/geometries/ConvexGeometry.js';
import * as BufferGeometryUtils from 'https://unpkg.com/three@0.145.0/examples/jsm/utils/BufferGeometryUtils.js';
import { AssetManager } from './AssetManager.js';
import { Stats } from "./stats.js";
import * as CSG from "./three-bvh-csg.js";
import { MeshBVH, ExtendedTriangle } from 'https://unpkg.com/three-mesh-bvh@0.5.10/build/index.module.js';
import MeshReflectorMaterial from "./MeshReflectorMaterial.js";
async function main() {
    // Setup basic renderer, controls, and profiler
    const clientWidth = window.innerWidth;
    const clientHeight = window.innerHeight;
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, clientWidth / clientHeight, 0.1, 1000);
    camera.position.set(0, 25, -50);
    const renderer = new THREE.WebGLRenderer();
    renderer.setSize(clientWidth, clientHeight);
    document.body.appendChild(renderer.domElement);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.VSMShadowMap;
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 25, 0);
    const stats = new Stats();
    stats.showPanel(0);
    document.body.appendChild(stats.dom);
    const ammo = await Ammo();
    let m_collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
    let m_dispatcher = new Ammo.btCollisionDispatcher(m_collisionConfiguration);
    let m_broadphase = new Ammo.btDbvtBroadphase();
    let m_constraintSolver = new Ammo.btSequentialImpulseConstraintSolver();

    const physicsWorld = new Ammo.btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);
    physicsWorld.setGravity(new Ammo.btVector3(0, -9.81, 0));

    Ammo.btGImpactCollisionAlgorithm.prototype.registerAlgorithm(physicsWorld.getDispatcher());
    let transformAux1 = new Ammo.btTransform();
    let tempBtVec3_1 = new Ammo.btVector3(0, 0, 0);
    const margin = 0.01;
    const rigidBodies = [];

    function createParalellepiped(sx, sy, sz, mass, pos, quat, material) {

        let threeObject = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz, 1, 1, 1), material);
        let shape = new Ammo.btBoxShape(new Ammo.btVector3(sx * 0.5, sy * 0.5, sz * 0.5));
        shape.setMargin(margin);

        createRigidBody(threeObject, shape, mass, pos, quat);

        return threeObject;

    }

    function createParalellepipedWithPhysics(sx, sy, sz, mass, pos, quat, material) {

        const object = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz, 1, 1, 1), material);
        const shape = new Ammo.btBoxShape(new Ammo.btVector3(sx * 0.5, sy * 0.5, sz * 0.5));
        shape.setMargin(margin);

        createRigidBody(object, shape, mass, pos, quat);

        return object;

    }

    function makeConvexHull(child, points = 32) {
        const modifier = new SimplifyModifier();
        const collisionMesh = child.clone();
        collisionMesh.geometry = collisionMesh.geometry.clone();
        // collisionMesh.material.flatShading = true;
        collisionMesh.geometry.deleteAttribute('normal');
        collisionMesh.geometry = BufferGeometryUtils.mergeVertices(collisionMesh.geometry);
        collisionMesh.geometry.computeVertexNormals();
        //const count = Math.floor(collisionMesh.geometry.attributes.position.count * 0.0); // number of vertices to remove
        // collisionMesh.geometry = modifier.modify(collisionMesh.geometry, count);
        let vertexPoses = [];
        let p = collisionMesh.geometry.attributes.position;
        for (let i = 0; i < p.count; i++) {
            vertexPoses.push(new THREE.Vector3(p.getX(i), p.getY(i), p.getZ(i)));
        }
        let hullGeo = new ConvexGeometry(vertexPoses);
        //scene.add(new THREE.Mesh(hullGeo, new THREE.MeshStandardMaterial({ color: new THREE.Color(1.0, 0.0, 0.0) })))
        //  hullGeo = BufferGeometryUtils.mergeVertices(hullGeo);
        // const count = hullGeo.attributes.position.count - points;
        //  hullGeo = modifier.modify(hullGeo.clone(), count);
        //scene.add(new THREE.Mesh(hullGeo.clone().translate(0, 15, 0), new THREE.MeshStandardMaterial()));
        vertexPoses = [];
        p = hullGeo.attributes.position;
        for (let i = 0; i < p.count; i++) {
            vertexPoses.push(new THREE.Vector3(p.getX(i), p.getY(i), p.getZ(i)));
        }
        if (vertexPoses.length === 0) {
            return null;
        }
        let trimesh = new Ammo.btConvexHullShape(); //createGImpactCollision(collisionMesh);
        vertexPoses.forEach(vertex => {
            trimesh.addPoint(new Ammo.btVector3(vertex.x, vertex.y, vertex.z), true);
        });
        return trimesh;
    }
    let pos = new THREE.Vector3();
    let quat = new THREE.Quaternion();

    function createRigidBody(object, physicsShape, mass, pos, quat, vel, angVel) {

        if (pos) {

            object.position.copy(pos);

        } else {

            pos = object.position;

        }

        if (quat) {

            object.quaternion.copy(quat);

        } else {

            quat = object.quaternion;

        }

        const transform = new Ammo.btTransform();
        transform.setIdentity();
        transform.setOrigin(new Ammo.btVector3(pos.x, pos.y, pos.z));
        transform.setRotation(new Ammo.btQuaternion(quat.x, quat.y, quat.z, quat.w));
        const motionState = new Ammo.btDefaultMotionState(transform);

        const localInertia = new Ammo.btVector3(0, 0, 0);
        physicsShape.calculateLocalInertia(mass, localInertia);

        const rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, motionState, physicsShape, localInertia);
        const body = new Ammo.btRigidBody(rbInfo);

        body.setFriction(0.5);

        if (vel) {

            body.setLinearVelocity(new Ammo.btVector3(vel.x, vel.y, vel.z));

        }

        if (angVel) {

            body.setAngularVelocity(new Ammo.btVector3(angVel.x, angVel.y, angVel.z));

        }

        object.userData.physicsBody = body;
        object.userData.collided = false;

        if (mass > 0) {

            rigidBodies.push(object);

            // Disable deactivation
            body.setActivationState(4);

        }

        physicsWorld.addRigidBody(body);

        return body;

    }
    // Setup scene
    // Skybox
    const environment = new THREE.CubeTextureLoader().load([
        "skybox/Box_Right.bmp",
        "skybox/Box_Left.bmp",
        "skybox/Box_Top.bmp",
        "skybox/Box_Bottom.bmp",
        "skybox/Box_Front.bmp",
        "skybox/Box_Back.bmp"
    ]);
    environment.encoding = THREE.sRGBEncoding;
    scene.background = environment;
    // Lighting
    const ambientLight = new THREE.AmbientLight(new THREE.Color(1.0, 1.0, 1.0), 0.05);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.35);
    directionalLight.position.set(150, 200, 50);
    // Shadows
    directionalLight.castShadow = true;
    directionalLight.shadow.mapSize.width = 1024;
    directionalLight.shadow.mapSize.height = 1024;
    directionalLight.shadow.camera.left = -75;
    directionalLight.shadow.camera.right = 75;
    directionalLight.shadow.camera.top = 75;
    directionalLight.shadow.camera.bottom = -75;
    directionalLight.shadow.camera.near = 0.1;
    directionalLight.shadow.camera.far = 500;
    directionalLight.shadow.bias = -0.001;
    directionalLight.shadow.blurSamples = 8;
    directionalLight.shadow.radius = 4;
    scene.add(directionalLight);
    const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.15);
    directionalLight2.color.setRGB(1.0, 1.0, 1.0);
    directionalLight2.position.set(-50, 200, -150);
    //scene.add(directionalLight2);
    // Objects
    const groundMap = await new THREE.TextureLoader().loadAsync("cuttingboard (1).jpeg");
    groundMap.encoding = THREE.sRGBEncoding;
    groundMap.wrapS = THREE.RepeatWrapping;
    groundMap.wrapT = THREE.RepeatWrapping;
    groundMap.repeat.set(1, 1);
    groundMap.anisotropy = renderer.capabilities.getMaxAnisotropy();
    const normalMap = await new THREE.TextureLoader().loadAsync("NormalMap (22).png");
    normalMap.wrapS = THREE.RepeatWrapping;
    normalMap.wrapT = THREE.RepeatWrapping;
    normalMap.repeat.set(1, 1);
    normalMap.anisotropy = renderer.capabilities.getMaxAnisotropy();
    const ground = createParalellepipedWithPhysics(100, 0.001, 100, 0, pos, quat, new THREE.Material());
    ground.material = new MeshReflectorMaterial(renderer, camera, scene, ground, {
        /*color: 0xFFFFFF,
        map: groundMap,
        normalMap: normalMap,
        envMap: environment,
        roughness: 0.1*/
        resolution: 1024,
        mixStrength: 1.0,
        reflectorOffset: 0.0,
        planeNormal: new THREE.Vector3(0, 1, 0),
        blur: [1024, 512]
    });
    ground.material.roughness = 0.25;
    ground.material.normalScale = new THREE.Vector2(0.25, 0.25);
    ground.material.map = groundMap;
    ground.material.normalMap = normalMap;
    ground.material.envMap = environment;
    ground.material.envMapIntensity = 0.5;
    ground.material.dithering = true;
    ground.material.specularIntensity = 0.4;
    scene.add(ground);
    ground.receiveShadow = true;
    const box = new THREE.Mesh(new THREE.BoxGeometry(10, 10, 10), new THREE.MeshStandardMaterial({ side: THREE.DoubleSide, color: new THREE.Color(1.0, 0.0, 0.0) }));
    box.castShadow = true;
    box.receiveShadow = true;
    box.position.y = 5.01;
    //scene.add(box);
    const sphere = new THREE.Mesh(new THREE.SphereGeometry(6.25, 32, 32), new THREE.MeshStandardMaterial({ side: THREE.DoubleSide, envMap: environment, metalness: 1.0, roughness: 0.25 }));
    sphere.position.y = 7.5;
    sphere.position.x = 25;
    sphere.position.z = 25;
    sphere.castShadow = true;
    sphere.receiveShadow = true;
    //scene.add(sphere);
    const torusKnot = new THREE.Mesh(new THREE.TorusKnotGeometry(5, 1.5, 200, 32), new THREE.MeshStandardMaterial({ side: THREE.DoubleSide, envMap: environment, metalness: 0.5, roughness: 0.5, color: new THREE.Color(0.0, 1.0, 0.0) }));
    torusKnot.position.y = 10;
    torusKnot.position.x = -25;
    torusKnot.position.z = -25;
    torusKnot.castShadow = true;
    torusKnot.receiveShadow = true;
    //scene.add(torusKnot);
    let dragon = (await new GLTFLoader().loadAsync("armadillochosen.glb")).scene;
    dragon.traverse(e => {
        if (e.isMesh) {
            dragon = e;
        }
    });
    // Create cube render target
    const cubeRenderTarget = new THREE.WebGLCubeRenderTarget(256, { generateMipmaps: true, minFilter: THREE.LinearMipmapLinearFilter });

    // Create cube camera
    const cubeCamera = new THREE.CubeCamera(1, 1000, cubeRenderTarget);
    scene.add(cubeCamera);
    dragon.geometry.scale(0.05, 0.05, 0.05);
    dragon.geometry.translate(0, 5, 0);
    dragon.castShadow = true;
    dragon.receiveShadow = true;
    dragon.material.normalMap = null;
    cubeCamera.position.copy((new THREE.Box3().setFromObject(dragon, true)).getCenter(new THREE.Vector3()));
    ground.material.roughness = 1.0;
    cubeCamera.update(renderer, scene);
    ground.material.roughness = 0.25;
    const dragonMat = new THREE.MeshPhysicalMaterial({
        transmission: 1.0,
        roughness: 0.1,
        thickness: 5.0,
        ior: 1.5,
        attenuationColor: new THREE.Color(0.25, 0.75, 0.25),
        //color: new THREE.Color(0.5, 1.0, 0.5),
        attenuationDistance: 5.0,
        side: THREE.DoubleSide,
        envMap: cubeRenderTarget.texture
    })
    dragon.material = dragonMat;
    //scene.background = cubeRenderTarget.texture;
    scene.add(dragon);
    createRigidBody(dragon, makeConvexHull(dragon), 15, dragon.position, dragon.quaternion);
    let bodies = [];
    bodies.push(dragon);
    // const plane = new THREE.Mesh(new THREE.PlaneGeometry(100, 100).rotateX(-Math.PI / 2).translate(0, 10, 0), new THREE.MeshStandardMaterial({ side: THREE.DoubleSide }));
    //scene.add(plane);
    // scene.add(dragon);
    const eviscerate = (mesh, plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -10)) => {
        if (!(new THREE.Box3().setFromObject(mesh, true)).intersectsPlane(plane)) {
            return;
        }
        scene.remove(mesh);
        if (mesh.userData.physicsBody) {
            physicsWorld.removeRigidBody(mesh.userData.physicsBody);
            //Ammo.system.driver.removeBody(dragon.userData.physicsBody);
        }
        bodies.splice(bodies.indexOf(mesh), 1);
        mesh.updateMatrixWorld();
        mesh.geometry.applyMatrix4(mesh.matrixWorld);
        mesh.geometry.computeBoundingBox();
        const csgEvaluator = new CSG.Evaluator();
        const planeProxyClip = new THREE.BoxGeometry(1000, 1000, 1000);
        planeProxyClip.translate(0, 0, 499.9 - plane.constant);
        planeProxyClip.lookAt(plane.normal);
        const planeProxyClipOther = new THREE.BoxGeometry(1000, 1000, 1000);
        planeProxyClipOther.translate(0, 0, 499.9 + plane.constant);
        planeProxyClipOther.lookAt(plane.normal.clone().multiplyScalar(-1));
        //scene.add(new THREE.Mesh(planeProxyClip, new THREE.MeshStandardMaterial({ color: new THREE.Color(1, 0, 0) })));
        //scene.add(new THREE.Mesh(planeProxyClipOther, new THREE.MeshStandardMaterial({ color: new THREE.Color(0, 1, 0) }))
        const topSlice = new CSG.Brush(planeProxyClip);
        const bottomSlice = new CSG.Brush(planeProxyClipOther);
        bottomSlice.geometry.computeVertexNormals();
        //scene.add(new THREE.Mesh(bottomSlice.geometry, new THREE.MeshStandardMaterial()));
        // const bottomSlice = new CSG.Brush(new THREE.BoxGeometry(100, 1000, 100).scale(0, -1, 0).translate(0, 10 + 500, 0));
        const meshBrush = new CSG.Brush(mesh.geometry);
        const resultTop = csgEvaluator.evaluate(meshBrush, topSlice, CSG.SUBTRACTION);
        const resultBottom = csgEvaluator.evaluate(meshBrush, bottomSlice, CSG.SUBTRACTION);
        const calcParts = (resultBottom) => {
            let parts = [];
            const posMap = {};
            const positions = resultBottom.geometry.attributes.position;
            if (positions.count === 0) {
                return [];
            }
            const normals = resultBottom.geometry.attributes.normal;
            const uvs = resultBottom.geometry.attributes.uv;
            const cache = {};
            const hashify = (i) => {
                if (cache[i]) {
                    return cache[i];
                }
                cache[i] = positions.getX(i).toFixed(2) + positions.getY(i).toFixed(2) + positions.getZ(i).toFixed(2);
                return cache[i];
            }
            for (let i = 0; i < positions.count; i += 3) {
                const vert1 = hashify(i);
                const vert2 = hashify(i + 1);
                const vert3 = hashify(i + 2);
                if (!posMap[vert1]) {
                    posMap[vert1] = [];
                }
                if (!posMap[vert2]) {
                    posMap[vert2] = [];
                }
                if (!posMap[vert3]) {
                    posMap[vert3] = [];
                }
                posMap[vert1].push(vert2, vert3);
                posMap[vert2].push(vert1, vert3);
                posMap[vert3].push(vert1, vert2);
            }
            let noPoses = {};
            while (Object.keys(noPoses).length < positions.count) {
                const availableIs = []; //Array(positions.length).fill()
                for (let i = 0; i < positions.count; i++) {
                    if (!noPoses[hashify(i)]) {
                        availableIs.push(i);
                    }
                }
                if (availableIs.length === 0) {
                    break;
                }
                const randomPos = hashify(availableIs[Math.floor(Math.random() * availableIs.length)]);
                const posesCollected = {};
                const posesToGoThrough = [randomPos];
                while (true) {
                    if (posesToGoThrough.length === 0) {
                        break;
                    }
                    const pos = posesToGoThrough.pop();
                    posesCollected[pos] = true;
                    const cs = posMap[pos];
                    for (let i = 0; i < cs.length; i++) {
                        if (!posesCollected[cs[i]]) {
                            posesToGoThrough.push(cs[i]);
                        }
                    }
                }
                const part = new THREE.BufferGeometry();
                const newPositions = [];
                const newNormals = [];
                const newUvs = [];
                const madeTries = {}
                for (let i = 0; i < positions.count; i += 3) {
                    const vert1 = hashify(i);
                    const vert2 = hashify(i + 1);
                    const vert3 = hashify(i + 2);
                    const combined = vert1 + vert2 + vert3;
                    if (posesCollected[vert1] && posesCollected[vert2] && posesCollected[vert3] && !madeTries[combined]) {
                        madeTries[vert1 + vert2 + vert3] = true;
                        madeTries[vert2 + vert3 + vert1] = true;
                        madeTries[vert3 + vert1 + vert2] = true;
                        noPoses[vert1] = true;
                        noPoses[vert2] = true;
                        noPoses[vert3] = true;
                        newPositions.push(positions.getX(i), positions.getY(i), positions.getZ(i));
                        newPositions.push(positions.getX(i + 1), positions.getY(i + 1), positions.getZ(i + 1));
                        newPositions.push(positions.getX(i + 2), positions.getY(i + 2), positions.getZ(i + 2));
                        newNormals.push(normals.getX(i), normals.getY(i), normals.getZ(i));
                        newNormals.push(normals.getX(i + 1), normals.getY(i + 1), normals.getZ(i + 1));
                        newNormals.push(normals.getX(i + 2), normals.getY(i + 2), normals.getZ(i + 2));
                        newUvs.push(uvs.getX(i), uvs.getY(i));
                        newUvs.push(uvs.getX(i + 1), uvs.getY(i + 1));
                        newUvs.push(uvs.getX(i + 2), uvs.getY(i + 2));
                    }
                }
                part.setAttribute("position", new THREE.BufferAttribute(new Float32Array(newPositions), 3));
                part.setAttribute("normal", new THREE.BufferAttribute(new Float32Array(newNormals), 3));
                part.setAttribute("uv", new THREE.BufferAttribute(new Float32Array(newUvs), 3));
                parts.push(part);
            }
            return parts;
        }
        const bottomParts = calcParts(resultTop);
        const topParts = calcParts(resultBottom);
        const allParts = [...topParts, ...bottomParts];
        const totalMass = 0.015;
        for (const part of allParts) {
            part.computeBoundingBox();
            part.boundingBox.getCenter(pos);
            part.center();
            const partMesh = new THREE.Mesh(part, dragonMat);
            partMesh.castShadow = true;
            partMesh.receiveShadow = true;
            const meshShape = makeConvexHull(partMesh);
            if (!meshShape) {
                continue;
            }
            scene.add(partMesh);
            createRigidBody(partMesh, meshShape, totalMass / allParts.length, pos, quat, new THREE.Vector3(0, topParts.includes(part) ? 17.5 : 0, 0), new THREE.Vector3(Math.random(), Math.random(), Math.random()));
            bodies.push(partMesh);
        }
    }
    const keys = {};
    document.onkeydown = (e) => {
        keys[e.key] = true;
        controls.enabled = false;
        /* if (e.key === "e") {
            bodies.forEach(body => {
                 eviscerate(body);
             })
         }*/
    }
    document.onkeyup = (e) => {
        keys[e.key] = false;
        controls.enabled = true;
    }
    const point1 = new THREE.Vector2();
    const point2 = new THREE.Vector2();

    function translatePointer(event) {

        // calculate pointer position in normalized device coordinates
        // (-1 to +1) for both components
        const pointer = new THREE.Vector2();
        pointer.x = (event.x / clientWidth) * 2 - 1;
        pointer.y = -(event.y / clientHeight) * 2 + 1;
        return pointer;
    }

    document.onmousedown = (e) => {
        if (keys["e"] === true) {
            if (point1.length() === 0) {
                point1.x = e.clientX;
                point1.y = e.clientY;
            } else {
                point2.x = e.clientX;
                point2.y = e.clientY;
                const raycaster1 = new THREE.Raycaster();
                const raycaster2 = new THREE.Raycaster();

                raycaster1.setFromCamera(translatePointer(point1), camera);
                raycaster2.setFromCamera(translatePointer(point2), camera);
                const rayPos1 = camera.position.clone().add(raycaster1.ray.direction);
                const rayPos2 = camera.position.clone().add(raycaster2.ray.direction);
                const rayOrigin1 = camera.position.clone().add(raycaster1.ray.direction.clone().multiplyScalar(camera.position.length()));
                const rayOrigin2 = camera.position.clone().add(raycaster2.ray.direction.clone().multiplyScalar(camera.position.length()));
                const rayDir1 = rayPos2.clone().sub(rayPos1).normalize();
                const rayDir2 = camera.getWorldDirection(new THREE.Vector3());
                raycaster1.setFromCamera(translatePointer(point1.clone().add(point2).multiplyScalar(0.5)), camera);
                const cameraSpace = camera.matrixWorld.elements;
                const right = new THREE.Vector3(cameraSpace[0], cameraSpace[1], cameraSpace[2]);
                const up = new THREE.Vector3(cameraSpace[4], cameraSpace[5], cameraSpace[6]);
                console.time();
                let ray1Hit = raycaster1.intersectObjects(scene.children); //[0];.point;
                let ray2Hit = raycaster2.intersectObjects(scene.children); //[0].point;
                console.timeEnd();
                if (ray1Hit.length > 0) {
                    ray1Hit = ray1Hit[0].point;
                } else {
                    ray1Hit = raycaster1.ray.origin.clone().add(raycaster1.ray.direction.clone().multiplyScalar(1000));
                }
                if (ray2Hit.length > 0) {
                    ray2Hit = ray2Hit[0].point;
                } else {
                    ray2Hit = raycaster2.ray.origin.clone().add(raycaster2.ray.direction.clone().multiplyScalar(1000));
                }

                const centroid = ray1Hit.clone().add(ray2Hit).multiplyScalar(0.5);
                const slicerPlane = new THREE.Plane().setFromCoplanarPoints(ray1Hit, ray2Hit, centroid.add(camera.position.clone().sub(centroid).normalize()));
                // Create a basic rectangle geometry
                const planeGeometry = new THREE.PlaneGeometry(100, 100);

                // Align the geometry to the plane
                const coplanarPoint = slicerPlane.coplanarPoint(new THREE.Vector3());
                const focalPoint = new THREE.Vector3().copy(coplanarPoint).add(slicerPlane.normal);
                const slash = new THREE.Mesh(planeGeometry, new THREE.ShaderMaterial({
                    side: THREE.DoubleSide,
                    uniforms: {
                        time: { value: 0.0 }
                    },
                    transparent: true,
                    depthTest: false,
                    vertexShader: `
                    uniform float time;
                    varying vec3 vPosition;
                    void main() {
                        vPosition = position;
                        gl_Position = projectionMatrix * viewMatrix * modelMatrix * vec4(position, 1.0);
                    }
                    `,
                    fragmentShader: `
                    uniform float time;
                    varying vec3 vPosition;
                    void main() {
                        float move = time * time * 0.01;
                        float param = vPosition.x - move;
                        float weight = 1.0 - smoothstep(0.0, 1.0, pow(move / 100.0, 1.5));
                        gl_FragColor = vec4(vec3(1.0), (1.0 / -(param + 50.0)) * weight);
                    }
                    `
                }));
                slash.lookAt(focalPoint);
                slash.rotateX(Math.PI / 12);
                slash.position.set(coplanarPoint.x, coplanarPoint.y, coplanarPoint.z);
                scene.add(slash);
                let startTime = -1;
                setInterval(() => {
                    if (startTime < 0) {
                        startTime = performance.now();
                    }
                    slash.material.uniforms.time.value = performance.now() - startTime;
                }, 1000 / 60);
                //console.log(slicerPlane);
                bodies.forEach(body => {
                    eviscerate(body, slicerPlane);
                })
                point1.x = 0;
                point1.y = 0;
                point2.x = 0;
                point2.y = 0;
                document.getElementById("lineVisualizer").innerHTML = "";
            }
        }
    }
    document.onmousemove = (e) => {
            if (point1.length() > 0) {
                document.getElementById("lineVisualizer").innerHTML = `
                <line x1="${point1.x}" y1="${point1.y}" x2="${e.clientX}" y2="${e.clientY}" style="stroke:rgb(255,0,0);stroke-width:2" />
                `
            }
        }
        // console.log(randomPos);
        // scene.add(new THREE.Mesh(resultTop.geometry, new THREE.MeshStandardMaterial({ /*normalMap: dragon.material.normalMap*/ })));
        //scene.add(new THREE.Mesh(resultBottom.geometry, new THREE.MeshStandardMaterial({ /*normalMap: dragon.material.normalMap*/ })));
        // Build postprocessing stack
        // Render Targets
    const defaultTexture = new THREE.WebGLRenderTarget(clientWidth, clientHeight, {
        minFilter: THREE.LinearFilter,
        magFilter: THREE.NearestFilter
    });
    defaultTexture.depthTexture = new THREE.DepthTexture(clientWidth, clientHeight, THREE.FloatType);
    // Post Effects
    const composer = new EffectComposer(renderer);
    const smaaPass = new SMAAPass(clientWidth, clientHeight);
    const effectPass = new ShaderPass(EffectShader);
    composer.addPass(effectPass);
    composer.addPass(new ShaderPass(GammaCorrectionShader));
    composer.addPass(smaaPass);

    function updatePhysics(deltaTime) {

        // Step world
        physicsWorld.stepSimulation(deltaTime, 10);

        // Update rigid bodies
        for (let i = 0, il = rigidBodies.length; i < il; i++) {

            const objThree = rigidBodies[i];
            const objPhys = objThree.userData.physicsBody;
            const ms = objPhys.getMotionState();

            if (ms) {

                ms.getWorldTransform(transformAux1);
                const p = transformAux1.getOrigin();
                const q = transformAux1.getRotation();
                objThree.position.set(p.x(), p.y(), p.z());
                objThree.quaternion.set(q.x(), q.y(), q.z(), q.w());

                objThree.userData.collided = false;

            }

        }
    }
    const clock = new THREE.Clock();

    function animate() {
        ground.material.roughness = 0.25;
        controls.update();
        ground.material.update();
        renderer.setRenderTarget(defaultTexture);
        renderer.clear();
        renderer.render(scene, camera);
        effectPass.uniforms["sceneDiffuse"].value = defaultTexture.texture;
        composer.render();
        stats.update();
        requestAnimationFrame(animate);
    }
    requestAnimationFrame(animate);
}
main();
