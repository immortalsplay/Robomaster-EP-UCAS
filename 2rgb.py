import os
from PIL import Image
from tqdm import tqdm
import numpy as np
 
img_path = (r"E:\1lab materials\\img") #填入图片所在文件夹的路径
img_Topath = (r"E:\1lab materials\\img") #填入图片转换后的文件夹路径
 
for item in tqdm(img_path):
    arr=item.strip().split('*')
    img_name=arr[0]
    image_path=os.path.join(img_path,img_name)
    img=Image.open(r"E:\1lab materials\\img\3.png")
    if(img.mode!='RGB'):        
        img = img.convert("RGB")
        img=np.array(img)
        print(img_name)
        print(img.shape)
        img.save(img_Topath +'/'+img_name,img)