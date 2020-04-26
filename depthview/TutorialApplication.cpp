/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.cpp
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#include "TutorialApplication.h"

#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS || OGRE_PLATFORM == OGRE_PLATFORM_APPLE
#   include <macUtils.h>
#   include "AppDelegate.h"
#endif
//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
    :depth(0)
{
	mTimer = OGRE_NEW Ogre::Timer();
	mTimer->reset();

    // Предварительно заполним калибровочную матрицу
    resetCalibration();

    // Помимо прочего нам нужно загрузить калибровачную матрица и сделать первоначальную калибровку сенсора
    // в калибровочной матрице содержится снимок глубин (сырой 11бит) эталонной плоскости.
    loadCalibration();
}
//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void)
{
    freenect_sync_stop();
}

void TutorialApplication::resetCalibration() {
    for (long i = 0; i < MAX_VERTEX_COUNT; i++) {
        calibration[i] = 1.0;
    }
}

void TutorialApplication::loadCalibration() {
    std::ifstream calibration_matrix;
    calibration_matrix.open("./matrix.clbt");
    if (calibration_matrix.is_open()) {
        unsigned long long sum = 0;
        unsigned int count_of_usable_depth = 0;
        unsigned short cur_depth = 2047;
        for (long i = 0; i < MAX_VERTEX_COUNT; i++) {
            calibration_matrix >> cur_depth;
            calibration[i] = cur_depth;
            if (cur_depth < 2047) {
                sum += cur_depth;
                count_of_usable_depth++;
            }
        }
        // Если количество калибруемых значений равно 632*480 тогда все в порядке и можно калибровать.
        if (count_of_usable_depth == 632*480) {
            double calibration_coef = (double)sum/(double)count_of_usable_depth;
            for (long i = 0; i < MAX_VERTEX_COUNT; i++) {
                calibration[i] = calibration_coef/calibration[i];
            }
        } else {
            std::cout << "Invalid calibration target" << "\n";
        }
    }

    calibration_matrix.close();
}

//-------------------------------------------------------------------------------------
void TutorialApplication::createScene(void)
{
    /* Зачем дополнительно создавать submesh пока не понял */
    mesh = Ogre::MeshManager::getSingleton().createManual("CustomMesh", "General");
    subMesh = mesh->createSubMesh();

    /* Подготавливаем структуру для трех вершин (треугольник у нас) */
    mesh->sharedVertexData = new Ogre::VertexData;
    mesh->sharedVertexData->vertexCount = MAX_VERTEX_COUNT;

    /* Получаем ссылку на дескриптор буфера (описывает структуру) */
    Ogre::VertexDeclaration *decl = mesh->sharedVertexData->vertexDeclaration;
    size_t offset = 0;

    /* первый элемент буфера - это сама вершина (ее координаты) */
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

    /* добавляем в буфер цвета вершин */
    decl->addElement(0, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR);

    /* Генерируем вертексный буфер по описанию, которое выше */
    vertexBuffer = Ogre::HardwareBufferManager::getSingleton().
        createVertexBuffer(offset, mesh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC);

    Ogre::uint32 red, green, blue;
    Ogre::RenderSystem* rs = Ogre::Root::getSingleton().getRenderSystem();
    rs->convertColourValue(Ogre::ColourValue(1,0,0,1), &red);
    rs->convertColourValue(Ogre::ColourValue(0,1,0,1), &green);
    rs->convertColourValue(Ogre::ColourValue(0,0,1,1), &blue);

    /* блокируем буфер на запись и берем указатель на него */
    float *pVertex = static_cast<float *>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL));

    // Заполняем буфер фейковыми данными (плоская поверхность)

    for (int i = 0; i < 480; i++) {
        for (int j = 0; j < 640; j++) {
            *pVertex++ = -j;
            *pVertex++ = -i;
            *pVertex++ = 0.0f;
            *(*(Ogre::uint32**)&pVertex)++ = green;
        }
    }

    /* разблокируем */
    vertexBuffer->unlock();

    /* Создаем буфер для индексов */

    indexBuffer = Ogre::HardwareBufferManager::getSingleton().
        createIndexBuffer(Ogre::HardwareIndexBuffer::IT_32BIT, MAX_INDECES_COUNT, Ogre::HardwareBuffer::HBU_STATIC);

    /* Связываем вертексы и созданные меш */
    mesh->sharedVertexData->vertexBufferBinding->setBinding(0, vertexBuffer);
    subMesh->useSharedVertices = true;

    /* Получаем блокировку на запись и пишем индексы в буфер */
    uint32_t *indices = static_cast<uint32_t *>(indexBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL));

    // Это используемое количество индексов
    long indexCount = 0;
    // При вычислении индексов исключаем последнюю строку
    for (unsigned long i = 0; i < MAX_VERTEX_COUNT - 640; i++) {
        // Восемь правых значений не используются и нам надо их исключить.
        // Вместе с этим самое крайнее справа значение тоже нужно исключить.
        // Потому что оно не будет являться базовым для построения треугольников
        if (i % 640 > 630) {
            continue;
        }

        *indices++ = i + 1;
        *indices++ = i;
        *indices++ = i + 640;

        *indices++ = i + 1 + 640;
        *indices++ = i + 1;
        *indices++ = i + 1 + 640 - 1;

        indexCount += 6;
    }

    /* записали - разблокировали */
    indexBuffer->unlock();

    subMesh->indexData->indexBuffer = indexBuffer;
    subMesh->indexData->indexCount = indexCount;
    subMesh->indexData->indexStart = 0;

    /* Если не объявить рамку, то огр не сможет правильно обсчитать сетку
     * и она будет видна лишь в корневой ноде (если ее туда прицепить),
     * а в дочерних - не будет.
     * Для этого можно зачитать http://www.ogre3d.org/forums/viewtopic.php?f=2&t=60200
     */
    mesh->_setBounds(Ogre::AxisAlignedBox(-2000, -2000, -2000, 2000, 2000, 2000));

    /* нарисовали - грузим */
    mesh->load();

    /*
     * А теперь нужно задефайнить материал.
     * Если этого не сделать, то новоиспеченный триангл будет выглядеть белым,
     * а не многоцветным как задумано выше
     */
    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create("Test/ColourTest", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT);

    /* Создаем ноду на базе того, что накодили выше. */
    Ogre::Entity *entity = mSceneMgr->createEntity("CustomEntity", "CustomMesh", "General");
    entity->setMaterialName("Test/ColourTest", "General");
    Ogre::SceneNode *node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    node->attachObject(entity);
    Ogre::Real meshBorderSize = entity->getBoundingRadius();
    node->scale(100/meshBorderSize, 100/meshBorderSize, 100/meshBorderSize);


    // create sphere in (0, 0, 0);
    Ogre::Entity *sphere = mSceneMgr->createEntity("sphere", Ogre::SceneManager::PT_SPHERE);
    Ogre::SceneNode *sphereNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    sphereNode->attachObject(sphere);
    sphereNode->setPosition(Ogre::Vector3(0,0,0));
    Ogre::Real radius = sphere->getBoundingRadius();
    sphereNode->scale(1.0/radius, 1.0/radius, 1.0/radius);

    mCamera->setPosition(Ogre::Vector3(0, 0, 10));
    mCamera->lookAt(Ogre::Vector3(0, 0, 50));
    mCamera->setPolygonMode(Ogre::PM_POINTS);
}

/**
 * Преобразует глубину в реальное значение (в миллиметрах)
 */
inline float TutorialApplication::rawDepthToMM(float depth_value) {
    /*
     * Если посмотреть на оба графика функций приближенного перевода виртуальных точек в миллиметры, то можно увидеть,
     * что на расстоянии больше 1080 график находится в области отрицательных значений.
     * А при значениях больше 1050 расстояние становится больше 10 метров, что нас не совсем устраивает.
     * Поэтому ограничим глубину 10ю метрами. Это значение приблизительно равно 1050
     */
    if (depth_value < 1050) {
        // Два варианта формулы преобразования глубины в миллиметры
        // Первая работает ощутимо быстрее, но она менее точна
        float depth = 1000.0 / (depth_value  * -0.0030711016 + 3.3309495161);
        //double depth = 123.6 * tan(depth_value / 2842.5 + 1.1863);
        return depth;
    }
    return 10000.0f;
}

/**
 * Преобразует виртуальную точку point в точку с реальными координатами (в миллиметрах)
 */
void TutorialApplication::virtualPointToReal(int x, int y, float rawDepth, float* point){

    float depth = rawDepthToMM(rawDepth);

    // Смотри http://russianpenguin.ru/2014/11/16/kinect-%D0%BE-%D0%B2%D0%BE%D1%81%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BB%D0%B5%D0%BD%D0%B8%D0%B8-%D0%BA%D0%BE%D0%BE%D1%80%D0%B4%D0%B8%D0%BD%D0%B0%D1%82-%D0%B8-%D0%B0%D0%B1%D0%B1%D0%B5%D1%80%D0%B0/
    //static double fx_d = 319.5/tan(58/2*PI/180);
    //static double fy_d = 0.747261047*239.5/tan(45/2*PI/180);
    // Объявим статиками чтобы не пересчитывать по сотне раз
    static float fx =  319.5/tan(28.5*M_PI/180);
    static float fy =  239.5/tan(21.5*M_PI/180);
    static float fx_inverse = 1.0f/fx;
    static float fy_inverse = 1.0f/fy;

    // Смещаем картинку в начало координат и выполняем преобразования
    point[2] = depth;
    point[1] = (y - /*479/2*/ 239.5) * depth * fy_inverse;
    point[0] = (x - /*639/2*/ 319.5) * depth * fx_inverse;
}

bool TutorialApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    uint32_t ts;
    Ogre::uint32 colour;
    Ogre::RenderSystem* rs = Ogre::Root::getSingleton().getRenderSystem();

    if (freenect_sync_get_depth((void**)&depth, &ts, 0, FREENECT_DEPTH_11BIT) >= 0) {

        /* блокируем буфер на запись и берем указатель на него */
        float *pVertex = static_cast<float *>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL));

        // Заполняем буфер фейковыми данными (плоская поверхность)
        // Решение неоптимальное. Сделано для наглядности

        // Нормируем метрицу
        for (int i = 0; i < 480; i++) {
            for (int j = 0; j < 640; j++) {
                long index = i*640+j;

                depthMatrix[index] = depth[index]*calibration[index];
            }
        }

        for (int i = 0; i < 480; i++) {
            for (int j = 0; j < 640; j++) {
                long index = i*640+j;

                // Зададим цвет точке в пространстве HSB
                float H = 1.0f - depthMatrix[index]/(float)FREENECT_DEPTH_RAW_MAX_VALUE;

                Ogre::ColourValue colourValue(0, 0, 0, 0);
                colourValue.setHSB(H, 1, 1);
                rs->convertColourValue(colourValue, &colour);

                // Записываем сконвертированную реальную точку по указателю pVertex
                // [0] => x, [1] -> y, [x] -> z
                virtualPointToReal(640-j, 480-i, depthMatrix[index], pVertex);
                pVertex += 3; // Мы только что записали три значения. Сместимся на следующий элемент
                // зададим посчитанный цвет
                *(*(Ogre::uint32**)&pVertex)++ = colour;
            }
        }

        /* разблокируем */
        vertexBuffer->unlock();

        // Теперь обрабатываем поверхность и строим сетку без разрывов
        /* Получаем блокировку на запись и пишем индексы в буфер */
        uint32_t *indices = static_cast<uint32_t *>(indexBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL));

        // Это используемое количество индексов
        long indexCount = 0;
        // При вычислении индексов исключаем последнюю строку
        for (unsigned long i = 0; i < MAX_VERTEX_COUNT - 640; i++) {
            // Восемь правых значений не используются и нам надо их исключить.
            // Вместе с этим самое крайнее справа значение тоже нужно исключить.
            // Потому что оно не будет являться базовым для построения треугольников
            if (i % 640 > 630) {
                continue;
            }

            if ((fabs(depthMatrix[i+1] - depthMatrix[i]) < 5
                && fabs(depthMatrix[i+1] - depthMatrix[i+640]) < 5
                && fabs(depthMatrix[i] - depthMatrix[i+640]) < 5)
                && (depthMatrix[i] < 1050 && depthMatrix[i+1] < 1050 && depthMatrix[i+640] < 1050)
            ) {
                *indices++ = i + 1;
                *indices++ = i;
                *indices++ = i + 640;
                indexCount += 3;
            }

            if ((fabs(depthMatrix[i+1 + 640] - depthMatrix[i+1]) < 5
                && fabs(depthMatrix[i+1+640] - depthMatrix[i+1+640-1]) < 5
                && fabs(depthMatrix[i+1] - depthMatrix[i+1+640-1]) < 5)
                && (depthMatrix[i+1+640] < 1050 && depthMatrix[i+1] < 1050 && depthMatrix[i+1+640-1] < 1050)
            ) {
                *indices++ = i + 1 + 640;
                *indices++ = i + 1;
                *indices++ = i + 1 + 640 - 1;
                indexCount += 3;
            }

        }

        /* записали - разблокировали */
        indexBuffer->unlock();

        subMesh->indexData->indexBuffer = indexBuffer;
        subMesh->indexData->indexCount = indexCount;
        subMesh->indexData->indexStart = 0;
    }

    return BaseApplication::frameRenderingQueued(evt);
}

/**
 * Обрабатываем нажатие на клавиатуру. В частности нам нужно сохранять сетку.
 */
bool TutorialApplication::keyPressed( const OIS::KeyEvent &arg ) {
    if (arg.key == OIS::KC_K)   // Сохраняем глубины нормирующей плоскости в файл
    {
        std::ofstream depth_file;
        depth_file.open ("./matrix.clbt");
        for (long i = 0; i < MAX_VERTEX_COUNT; i++) {
            depth_file << depth[i] << " ";
        }
        depth_file.close();
    } else if (arg.key == OIS::KC_L) {
        loadCalibration();
    } else if (arg.key == OIS::KC_J) {
        resetCalibration();
    }

    return BaseApplication::keyPressed(arg);
}
